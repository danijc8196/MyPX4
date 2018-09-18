/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ekf2_control.cpp
 *
 * Code for controlling the ekf2 builtin module in px4.
 * It allows to start and stop the ekf2 module, and provides a new functionality: ekf2 reset.
 * Furthermore, this module has two interfaces: by commands and by mavlink messages.
 *
 * @author Daniel Jáuregui <danijc8196@gmail.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <matrix/math.hpp>


/* Prototypes */


/**
 * Daemon management function.
 *
 * This function allows to start / stop the background task (daemon).
 * The purpose of it is to be able to start the controller on the
 * command line, query its status and stop it, without giving up
 * the command line to one particular process or the need for bg/fg
 * ^Z support by the shell.
 */
extern "C" __EXPORT int ekf2_control_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int daemon_thread(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);


/* Variables */
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

/*  */

/* Main Thread (Daemon) */
int daemon_thread(int argc, char *argv[])
{
    /*
     * Open mavlink connection and wait for custom messages of start, stop, reset
     *
     */
    return 0;
}


/* Startup Functions */

static void usage(const char *reason)
{
        if (reason) {
                fprintf(stderr, "%s\n", reason);
        }

        fprintf(stderr, "usage: ekf2_control {start|stop|status|reset}\n\n");
        return;
}


int ekf2_control_main(int argc, char *argv[])
{
        if (argc < 2) {
                usage("missing command");
        }

        if (!strcmp(argv[1], "start")) {

                if (thread_running) {
                        printf("ekf2_control already running\n");
                        /* this is not an error */
                        return 0;
                }

                thread_should_exit = false;
                deamon_task = px4_task_spawn_cmd("ekf2_control",
                                                 SCHED_DEFAULT,
                                                 SCHED_PRIORITY_MAX - 20,
                                                 2048,
                                                 daemon_thread,
                                                 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
                thread_running = true;

                // Ekf2 start


                return 0;
        }

        if (!strcmp(argv[1], "stop")) {
                thread_should_exit = true;
                return 0;
        }

        if (!strcmp(argv[1], "status")) {
                if (thread_running) {
                        printf("\tekf2_control is running\n");

                } else {
                        printf("\tekf2_control not started\n");
                }

                return 0;
        }

        if (!strcmp(argv[1], "reset")) {
                if (thread_running) {
                    //stop ekf2
                    //start ekf2
                } else {
                    //start ekf2
                }
                return 0;
        }

        usage("unrecognized command");
        return 1;
}
