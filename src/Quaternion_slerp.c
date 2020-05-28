// Copyright (C) 2020 Myoungki Jung <lunarpulse@gmail.com>
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "../include/Quaternion_slerp.h"

#define TO_RAD(x) (x / 180.0 * M_PI)

double *cvt(char *input, int *level)
{
    char *cp = strtok(input, ", ");
    if (cp == NULL) {
        /* No more separators */
        return (double *) malloc(sizeof(double) * *level);
    }

    int my_index = -1;
    double n;
    if (sscanf(cp, "%lf", &n) == 1) {
        my_index = *level;
        *level += 1;
    } else {
        printf("Invalid double token '%s'\n", cp);
    }
    double *array = cvt(NULL, level);
    if (my_index >= 0) {
        array[my_index] = n;
    }
    return array;
}

int main(int argc, char *argv[])
{
    int debug = 0;
	extern char *optarg;
	extern int optind;
	int c, err = 0; 
	int tflag=0, fflag=0;
	char *fname;
    int stepCount=0;
    char *startQuat = "0.0, 0.0, 0.0, 0.0", *endQuat = "0.0, 0.0, 0.0, 0.0";
	static char usage[] = "usage: %s [-d] -f fname -t count [-a \"0.0, 0.0, 0.0, 0.0\"] [-b \"0.0, 0.0, 0.0, 0.0\"] [-s sname] name [name ...]\n";

	while ((c = getopt(argc, argv, "dft:a:b:")) != -1)
		switch (c) {
        case 'd':
            debug = 1;
            break;
		case 'f':
			fflag = 1;
			fname = optarg;
			break;
        case 't':  // step count
			tflag = 1;
			stepCount = atoi(optarg);
			break;
        case 'a': // start quat
			startQuat = optarg;
			break;
        case 'b': // end quat
			endQuat = optarg;
			break;
		case '?':
			err = 1;
			break;
		}

	if (fflag == 0 || tflag == 0) {	/* -f and -t was mandatory filename*/
		fprintf(stderr, "%s: missing -f or -t option\n", argv[0]);
		fprintf(stderr, usage, argv[0]);
		exit(1);
	} else if ((optind+1) > argc) {	
		/* need at least one argument (change +1 to +2 for two, etc. as needeed) */

		printf("optind = %d, argc=%d\n", optind, argc);
		fprintf(stderr, "%s: missing name\n", argv[0]);
		fprintf(stderr, usage, argv[0]);
		exit(1);
	} else if (err) {
		fprintf(stderr, usage, argv[0]);
		exit(1);
	}
    
	/* see what we have */
    if (debug == 1)
    {   
        printf("fname = \"%s\"\n", fname);
        printf("startQuat = \"%s\"\n", startQuat);
        printf("endQuat = \"%s\"\n", endQuat);    
    }

	if (optind < argc)	/* these are the arguments after the command-line options */
		for (; optind < argc; optind++)
        {
            if (debug == 1)
            {
			    printf("argument: \"%s\"\n", argv[optind]);
            }
        }

	else {
		printf("no arguments left to process\n");
	}

    int n_array = 0;
    double *array = cvt(startQuat, &n_array);
    double position[3] = {0, 0, 0};
    Quaternion orientation;
    //Quaternion_setIdentity(&orientation);   // The identity quaternion represents no rotation
    Quaternion_set(array[0], array[1], array[2], array[3], &orientation);
    if (debug == 1)
    {   
        printf("orientation = ");
        fprintf(stdout, "% 011.6f, % 011.6f, % 011.6f, % 011.6f\n",
        orientation.w, orientation.v[0], orientation.v[1], orientation.v[2]);
    }

    // Rotate character to the left (90 degrees around z)
    Quaternion rotated_orientation_apply;
    // double angle = TO_RAD(90.0);             // Rotation angle in radians
    // Quaternion_fromZRotation(angle, &rotated_orientation_apply);   // Set rotated_orientation_apply to represent the Z-rotation
    n_array = 0;
    array = cvt(endQuat, &n_array);
    Quaternion_set(array[0], array[1], array[2], array[3], &rotated_orientation_apply);
    if (debug == 1)
    {   
        printf("rotated_orientation_apply = ");
        fprintf(stdout, "% 011.6f, % 011.6f, % 011.6f, % 011.6f\n",
        rotated_orientation_apply.w, rotated_orientation_apply.v[0], rotated_orientation_apply.v[1], rotated_orientation_apply.v[2]);
    }

    // Walks half a circle with 10000 steps
    const double STEP_COUNT = stepCount;                     // Walk 10000 steps in half circle
    const double STEP_SIZE  = M_PI / 100.0;                    // Each step moves 3.14cm
    const double TIME_STEP = 1.0 / STEP_COUNT;

    Quaternion startRotation, endRotation;
    startRotation = orientation;                                  // Start facing left
    Quaternion_multiply(&rotated_orientation_apply, &orientation, &endRotation);

    if (debug == 1)
    { 
        Quaternion result;
        Quaternion_slerp(&startRotation, &endRotation, 0.0, &result);
        fprintf(stdout, "% 011.6f, % 011.6f, % 011.6f, % 011.6f\n",
            result.w, result.v[0], result.v[1], result.v[2]);
        Quaternion_slerp(&startRotation, &endRotation, 1.0, &result);
        fprintf(stdout, "% 011.6f, % 011.6f, % 011.6f, % 011.6f\n",
            result.w, result.v[0], result.v[1], result.v[2]);
    }

    double t = 0;
    while(t < (1+ TIME_STEP)) {
        Quaternion_slerp(&startRotation, &endRotation, t, &orientation);

        double step[3] = {STEP_SIZE, 0, 0};
        Quaternion_rotate(&orientation, step, step);   // Calculate step in character coordinate system
        position[0] += step[0];                        // Step forward in current orientation
        position[1] += step[1];
        position[2] += step[2];

        t += TIME_STEP;

        fprintf(stdout, "% 011.6f, % 011.6f, % 011.6f, % 011.6f\n",
        orientation.w, orientation.v[0], orientation.v[1], orientation.v[2]);
    }

    return EXIT_SUCCESS;
}
