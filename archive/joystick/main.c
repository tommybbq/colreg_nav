/* Adapted from https://archives.seul.org/linuxgames/Aug-1999/msg00107.html
 * Done by Evan Dunbar for Team Sea++
 */

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

// replace with correct path if different for some reason
#define JOY_DEV "/dev/input/js0"

// in microseconds
#define ACCEPTABLE_INPUT_DELAY 100

int main()
{
    int joy_fd, *axis=NULL, num_of_axis=0, num_of_buttons=0, i;
    char *button=NULL, name_of_joystick[80];
    struct js_event js;

    // try to open the device file
    if ((joy_fd = open(JOY_DEV, O_RDONLY)) == -1)
    {
        printf("Couldn't open joystick\n");
        return -1;
    }

    // file has been opened
    // read device info from file
    ioctl(joy_fd, JSIOCGAXES, &num_of_axis);
    ioctl(joy_fd, JSIOCGBUTTONS, &num_of_buttons);
    ioctl(joy_fd, JSIOCGNAME(80), &name_of_joystick);

    // variable size storage for axis and button values
    axis = (int *) calloc(num_of_axis, sizeof(int));
    button = (char *) calloc(num_of_buttons, sizeof(char));

    printf("Joystick detected: %s\n\t%d axis\n\t%d buttons\n\n",
           name_of_joystick,
           num_of_axis,
           num_of_buttons);

    // use non-blocking mode
    fcntl(joy_fd, F_SETFL, O_NONBLOCK);

    while(1) // infinite loop
    {
        // read joystick state
        read(joy_fd, &js, sizeof(struct js_event));

        // only read values after initialization
        switch (js.type & ~JS_EVENT_INIT)
        {
            case JS_EVENT_AXIS:
                axis[js.number] = js.value;
                break;
            case JS_EVENT_BUTTON:
                button[js.number] = js.value;
                break;
        }

        // print out new results
        for(i=0; i<num_of_axis; ++i)
            printf("A%d: %6d  ", i, axis[i]);

        for(i=0; i<num_of_buttons; ++i)
            printf("B%d: %d  ", i, button[i]);

        printf("  \r");
        fflush(stdout);

        // need this to prevent very high cpu usage
        usleep(ACCEPTABLE_INPUT_DELAY);
    }

    // never reached
    close(joy_fd);
    return 0;
}
