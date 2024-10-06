#ifndef dronev2_tasker_h
#define dronev2_tasker_h

// Requirements
#include <functional>
#include <vector>
#include <Arduino.h>

namespace dronev2
{
    class task
    {
    private:
        unsigned long previous;
        std::function<void()> executable;

    public:
        int interval;
        int task_id = random(0, 1000);

        task(std::function<void()> callback, int ms) : previous(millis()), interval(ms), executable(callback) {}
        task(std::function<void()> callback, int ms, int _id) : previous(millis()), interval(ms), executable(callback), task_id(_id) {}

        int execute();
    };

    class thread
    {
    private:
        std::vector<task> _tasks;

    public:
        template <typename... Tasks>
        thread(Tasks... tasks_)
        {
            (_tasks.push_back(tasks_), ...);
        }

        int add_task(task task_);
        int remove_task(int task_id);

        int update_task_interval(int task_id, int new_interval);

        int clear();
        void run();
    };

    class thread_ptr
    {
    private:
        std::vector<task *> _tasks;

    public:
        template <typename... Tasks>
        thread_ptr(Tasks... tasks_)
        {
            (_tasks.push_back(tasks_), ...);
        }

        int add_task(task *task_);
        int remove_task(int task_id);

        int update_task_interval(int task_id, int new_interval);

        int clear();
        void run();
    };
}

#endif