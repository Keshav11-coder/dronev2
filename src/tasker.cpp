// Requirements
#include <functional>
#include <vector>
#include <Arduino.h>

// Header
#include "tasker.h"

namespace dronev2
{
    int task::execute()
    {
        unsigned long currentMillis = millis();
        if (currentMillis - previous >= interval)
        {
            previous = currentMillis;
            executable();
            return 0;
        }
        return -1;
    }

    int thread::add_task(task task_)
    {
        _tasks.push_back(task_);
        return 0;
    }

    int thread::remove_task(int task_id)
    {
        for (int i = 0; i < _tasks.size(); i++)
        {
            if (_tasks[i].task_id == task_id)
            {
                _tasks.erase(_tasks.begin() + i);
                return 0;
            }
        }
        return 1;
    }

    int thread::update_task_interval(int task_id, int new_interval)
    {
        for (task task_ : _tasks)
        {
            if (task_.task_id == task_id)
            {
                task_.interval = new_interval;
            }
        }
        return 0;
    }

    int thread::clear()
    {
        _tasks.clear();
        return 0;
    }

    void thread::run()
    {
        for (int i = 0; i < _tasks.size(); i++)
        {
            _tasks[i].execute();
        }
    }

    int thread_ptr::add_task(task *task_)
    {
        _tasks.push_back(task_);
        return 0;
    }

    int thread_ptr::remove_task(int task_id)
    {
        for (int i = 0; i < _tasks.size(); i++)
        {
            if (_tasks[i]->task_id == task_id)
            {
                _tasks.erase(_tasks.begin() + i);
                return 0;
            }
        }
        return 1;
    }

    int thread_ptr::update_task_interval(int task_id, int new_interval)
    {
        for (task *task_ : _tasks)
        {
            if (task_->task_id == task_id)
            {
                task_->interval = new_interval;
            }
        }
        return 0;
    }

    int thread_ptr::clear()
    {
        for (task *ptr : _tasks)
        {
            delete ptr;
        }
        _tasks.clear();
        return 0;
    }

    void thread_ptr::run()
    {
        for (int i = 0; i < _tasks.size(); i++)
        {
            _tasks[i]->execute();
        }
    }
}