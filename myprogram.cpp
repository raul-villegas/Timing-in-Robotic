#include <iostream>
#include <chrono>
#include <thread>
#include <time.h>
#include <native/task.h>
#include <native/timer.h>

#define TASK_PERIOD 1000000 // 1 ms

void periodic_task(void *)
{
    RT_TASK task;
    rt_task_create(&task, "periodic_task", 0, 50, 0);
    rt_task_set_periodic(&task, TM_NOW, TASK_PERIOD);

    rt_printf("Starting periodic task\n");

    const int num_iterations = 10000000;
    double iteration_durations[num_iterations];

    timespec prev_end_time;
    clock_gettime(CLOCK_MONOTONIC, &prev_end_time);

    for (int i = 0; i < num_iterations; i++)
    {
        // Do some computational work
        double x = i * i;

        timespec iteration_end_time;
        clock_gettime(CLOCK_MONOTONIC, &iteration_end_time);

        double iteration_duration = (iteration_end_time.tv_sec - prev_end_time.tv_sec) * 1000.0 + (iteration_end_time.tv_nsec - prev_end_time.tv_nsec) / 1000000.0;

        iteration_durations[i] = iteration_duration;

        prev_end_time = iteration_end_time;

        // Sleep until the next execution time
        rt_task_wait_period(NULL);
    }

    rt_printf("Periodic task finished\n");

    // Print the iteration durations
    for (int i = 0; i < num_iterations; i++)
    {
        rt_printf("Iteration %d duration: %f ms\n", i, iteration_durations[i]);
    }

    rt_task_delete(&task);
}

int main()
{
    RT_TASK task;
    rt_task_create(&task, "main_task", 0, 99, 0);
    rt_task_start(&task, &periodic_task, NULL);
    rt_task_join(&task);
    rt_task_delete(&task);

    return 0;
}
