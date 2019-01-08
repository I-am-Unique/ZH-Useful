/**************************************************************************
 * @author z.h
 * @date 2019.1.7
 * @usage:yolo_ros.cpp 
 **************************************************************************/
#include "darknet_ros/yolo_ros.hpp"



yolo_ros::yolo_ros()
{
	ROS_INFO("Yolo Node Started~");
}

yolo_ros::~yolo_ros()
{
	free_image(buff_);
    delete []predictions_;
    delete []avg_;
    delete []roiBoxes_;
	ROS_INFO("Yolo Node Ended~");
	ros::shutdown();
}

void yolo_ros::init()
{
	//Set common parameters

	// Initialize deep network of darknet.
    n.param("/darknet_ros/yolo/threshold/value", thresh, (float) 0.5);

	std::string config_file;
    std::string weight_file;
    n.param("/darknet_ros/weight_file", weight_file, std::string("/default"));
    n.param("/darknet_ros/config_file", config_file, std::string("/default"));
    
    // Path to data folder.
    std::string curr_folder;
    n.param("/darknet_ros/curr_folder", curr_folder, std::string(""));
    std::string str = curr_folder + "/../darknet/data";
    demoAlphabet_ = load_alphabet_with_file((char*)str.c_str());

    //Get classes.
    n.param("/darknet_ros/yolo/detection_classes/names", classLabels_, std::vector<std::string>(0));
    numClasses_ = classLabels_.size();
    demoClasses_ = numClasses_;
    demoNames_ = (char**) realloc((void*) demoNames_, (demoClasses_ + 1) * sizeof(char*));
    for (int i = 0; i < demoClasses_; i++) {
	    demoNames_[i] = new char[classLabels_[i].length() + 1];
	    strcpy(demoNames_[i], classLabels_[i].c_str());
    }

    // Load network.
    net_ = load_network((char*)config_file.c_str(),
                        (char*)weight_file.c_str(), 0);
    set_batch_network(net_, 1); 
       
    // Initialize image publisher.
    n.param("/darknet_ros/ros/image_sub_name", image_sub_name_, std::string("/default"));

    // Open up space
    buff_.h = 480;
    buff_.w = 960;
    buff_.c = 3;
    buff_.data = (float*)calloc(buff_.h * buff_.w * buff_.c, sizeof(float));

    demoTotal_ = sizeNetwork(net_);
    predictions_ = (float *) calloc(demoTotal_, sizeof(float));
    avg_ = (float *) calloc(demoTotal_, sizeof(float));

    layer l = net_->layers[net_->n - 1];
    roiBoxes_ = (RosBox_ *) calloc(l.w * l.h * l.n, sizeof(RosBox_));

    // Box vector
    rosBoxes_ = std::vector < std::vector < RosBox_ > > (numClasses_);
    rosBoxCounter_ = std::vector<int>(numClasses_);
}

image **yolo_ros::load_alphabet_with_file(char *datafile) 
{
	int i, j;
	const int nsize = 8;
	image **alphabets = (image **)calloc(nsize, sizeof(image));
	char* labels = "/labels/%d_%d.png";
	char * files = (char *) malloc(1 + strlen(datafile)+ strlen(labels) );
	strcpy(files, datafile);
	strcat(files, labels);
	for(j = 0; j < nsize; ++j){
		alphabets[j] = (image*)calloc(128, sizeof(image));
		for(i = 32; i < 127; ++i){
			char buff[256];
			sprintf(buff, files, i, j);
			alphabets[j][i] = load_image_color(buff, 0, 0);
		}
	}
	return alphabets;
}


int yolo_ros::sizeNetwork(network *net)
{
    int i;
    int count = 0;
    for (i = 0; i < net->n; ++i)
    {
        layer l = net->layers[i];
        if (l.type == YOLO || l.type == REGION || l.type == DETECTION)
        {
            count += l.outputs;
        }
    }
    return count;
}


void yolo_ros::rememberNetwork(network *net)
{
    int i;
    int count = 0;
    for (i = 0; i < net->n; ++i)
    {
        layer l = net->layers[i];
        if (l.type == YOLO || l.type == REGION || l.type == DETECTION)
        {
            memcpy(predictions_ + count, net->layers[i].output, sizeof(float) * l.outputs);
            count += l.outputs;
        }
    }
}

detection *yolo_ros::avgPredictions(network *net, int *nboxes)
{
    int i, j;
    int count = 0;
    fill_cpu(demoTotal_, 0, avg_, 1);
    axpy_cpu(demoTotal_, 1, predictions_, 1, avg_, 1);
    for (i = 0; i < net->n; ++i)
    {
        layer l = net->layers[i];
        if (l.type == YOLO || l.type == REGION || l.type == DETECTION)
        {
            memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
            count += l.outputs;
        }
    }
    detection *dets = get_network_boxes(net, buff_.w, buff_.h, thresh, 0, 0, 1, nboxes);
    return dets;
}

int yolo_ros::fromMatToImage(const cv::Mat& mat, const image& out)
{
    int w = mat.cols;
    int h = mat.rows;
    int c = mat.channels();
    int wh = w * h;
    //image out = make_image(mat.cols, mat.rows, mat.channels());
    for (size_t i = 0; i < h; ++i)
    {
        for (size_t j = 0; j < w; ++j)
        {
            for (size_t k = 0; k < c; ++k)
            {
                out.data[k * wh + i * w + j] = mat.at<cv::Vec3b>(i, j).val[k] / 255.0f;
            }
        }
    }
    return 0;
}

sensor_msgs::Image yolo_ros::fromImageToRosImage(const image& image)
{
    sensor_msgs::Image sensor_img;
    int h = image.h;
    int w = image.w;
    int c = image.c;
    int wh = h * w;

    sensor_img.header.stamp = ros::Time::now();
    sensor_img.height = image.h;
    sensor_img.width = image.w;
    sensor_img.encoding = sensor_msgs::image_encodings::BGR8;
    sensor_img.data.resize(image.h * image.w * image.c);

    for (size_t i = 0; i < h; ++i)
    {
        for (size_t j = 0; j < w; ++j)
        {
            for (size_t k = 0; k < c; ++k)
            {
                sensor_img.data[i * w * c + j * c + k] =
                    image.data[k * wh + i * w + j] * 255.0;
            }
        }
    }

    return sensor_img;
}

void *yolo_ros::detect_yolo()
{
    float nms = .4;

    layer l = net_->layers[net_->n - 1];
    float *X = buffLetter_.data;

    ros::Time t_final = ros::Time::now();
    network_predict(net_, X);
    ROS_WARN("[darknet_ros][detectYolo] computation cost for yolo = %fs",
              (ros::Time::now() - t_final).toSec());

    rememberNetwork(net_);
    detection *dets = 0;
    int nboxes = 0;
    dets = avgPredictions(net_, &nboxes);
    /*
	ROS_WARN("yolo find %f boxes", nboxes);
	for (int i = 0; i < nboxes; ++i)
	{
	   ROS_INFO("%f, %f, %f, %f", dets[i].bbox.x, dets[i].bbox.y,
	            dets[i].bbox.w, dets[i].bbox.h);
	}
	*/
    if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

    image display = buff_;
    draw_detections(display, dets, nboxes, thresh, demoNames_, demoAlphabet_, demoClasses_);

    sensor_msgs::Image img = fromImageToRosImage(display);
    detection_img_pub.publish(img);

    // extract the bounding boxes and send them to ROS
    int i, j;
    int count = 0;
    for (i = 0; i < nboxes; ++i)
    {
        float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
        float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
        float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
        float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

        if (xmin < 0)
            xmin = 0;
        if (ymin < 0)
            ymin = 0;
        if (xmax > 1)
            xmax = 1;
        if (ymax > 1)
            ymax = 1;
        // iterate through possible boxes and collect the bounding boxes
        for (j = 0; j < demoClasses_; ++j)
        {
            if (dets[i].prob[j])
            {
                float x_center = (xmin + xmax) / 2;
                float y_center = (ymin + ymax) / 2;
                float BoundingBox_width = xmax - xmin;
                float BoundingBox_height = ymax - ymin;
                // define bounding box
                // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
                if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01)
                {
                    roiBoxes_[count].x = x_center;
                    roiBoxes_[count].y = y_center;
                    roiBoxes_[count].w = BoundingBox_width;
                    roiBoxes_[count].h = BoundingBox_height;
                    roiBoxes_[count].Class = j;
                    roiBoxes_[count].prob = dets[i].prob[j];
                    count++;
                }
            }
        }
    }
    // create array to store found bounding boxes
    // if no object detected, make sure that ROS knows that num = 0
    if (count == 0)
    {
        roiBoxes_[0].num = 0;
    } else
    {
        roiBoxes_[0].num = count;
    }
    //Only the categories(classLabels_) we want are shown here.
    int num = roiBoxes_[0].num;
    for (int i = 0; i < num; i++)
    {
        for (int j = 0; j < numClasses_; j++)
        {
            if (roiBoxes_[i].Class == j)
            {
                rosBoxes_[j].push_back(roiBoxes_[i]);
                rosBoxCounter_[j]++;//Number of target b boxes for each category
            }
        }
    }
    for (int i = 0; i < numClasses_; i++)
    {
        if (rosBoxCounter_[i] > 0)
        {
            perception_msgs::Object object;
            for (int j = 0; j < rosBoxCounter_[i]; j++)
            {
                int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * 960;
                int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * 480;
                int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * 960;
                int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * 480;

                object.bounding_box_center.x = (xmin + xmax) / 2;
                object.bounding_box_center.y = (ymin + ymax) / 2;
                object.bounding_box_size.x = xmax - xmin;
                object.bounding_box_size.y = ymax - ymin;
                object.classification = i;
                object.classification_certainty = rosBoxes_[i][j].prob * 100;

                objects_.objects.push_back(object);
            }
        }
    }

    objects_.header.stamp = ros::Time::now();
    objects_.header.frame_id = "objects";
    objects_pub.publish(objects_);
    //clear
    free_detections(dets, nboxes);
    objects_.objects.clear();
    for (int i = 0; i < numClasses_; i++)
    {
        rosBoxes_[i].clear();
        rosBoxCounter_[i] = 0;
    }
    return 0;

}

void *yolo_ros::yolo(const cv::Mat& im)
{
	fromMatToImage(im, buff_);
	buffLetter_ = letterbox_image(buff_, net_->w, net_->h);
	detect_yolo();
	free_image(buffLetter_);
}

void yolo_ros::cameraCallback(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
	ROS_DEBUG("Yolo object detector image is received!");

	cv::Mat raw_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);
    cv::cvtColor(raw_image, raw_image, cv::COLOR_BayerBG2BGR);
    cv::resize(raw_image, raw_image, cv::Size(960, 600));
    ros::Time t_final = ros::Time::now();
    yolo(raw_image(cv::Rect(0, 120, 960, 480)));
    ROS_WARN("[darknet_ros][cameraCallback] overall computation cost = %fs",
              (ros::Time::now() - t_final).toSec());    

}

void yolo_ros::doListening()
{
	ROS_INFO("begin to listen!");

	rawimage_sub = n.subscribe(image_sub_name_, 1, &yolo_ros::cameraCallback, this);
    detection_img_pub = n.advertise<sensor_msgs::Image>("/darknet_ros/detection_img", 1);
    objects_pub = n.advertise<perception_msgs::Objects>("/darknet_ros/objects", 1);
    ros::spin();

    ROS_INFO("stop to listen!");
}