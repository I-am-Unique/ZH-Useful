#ifndef THRAEDSAFE_STACK_HPP
#define THRAEDSAFE_STACK_HPP

#include <mutex>
#include <condition_variable>
#include <stack>
#include <memory>

template<typename T>
class threadsafe_stack
{
private:
    mutable std::mutex mut;
    std::stack<T> data_stack;
    std::condition_variable data_cond;
public:
    threadsafe_stack()
    {}
    threadsafe_stack(threadsafe_stack const& other)
    {
        std::lock_guard<std::mutex> lk(other.mut);
        data_stack=other.data_stack;
    }

    void push(T new_value)
    {
        std::lock_guard<std::mutex> lk(mut);
        data_stack.push(new_value);
        data_cond.notify_one();
    }

    void wait_and_pop(T& value)
    {
        std::unique_lock<std::mutex> lk(mut);
        data_cond.wait(lk,[this]{return !data_stack.empty();});
        value=data_stack.top();
        data_queue.pop();
    }

    std::shared_ptr<T> wait_and_pop()
    {
        std::unique_lock<std::mutex> lk(mut);
        data_cond.wait(lk,[this]{return !data_stack.empty();});
        std::shared_ptr<T> res(std::make_shared<T>(data_stack.top()));
        data_queue.pop();
        return res;
    }

    bool try_pop(T& value)
    {
        std::lock_guard<std::mutex> lk(mut);
        if(data_stack.empty())
            return false;
        value=data_stack.top();
        data_stack.pop();
        return true;
    }

    std::shared_ptr<T> try_pop()
    {
        std::lock_guard<std::mutex> lk(mut);
        if(data_stack.empty())
            return std::shared_ptr<T>();
        std::shared_ptr<T> res(std::make_shared<T>(data_stack.top()));
        data_stack.pop();
        return res;
    }

    bool empty() const
    {
        std::lock_guard<std::mutex> lk(mut);
        return data_stack.empty();
    }
};

////////////////////////////
//threadsafe_stack<View> ttt;
//ttt.push(view);

/////redar//

//View view2;
//bool res = ttt.try_pop(view2);

#endif
