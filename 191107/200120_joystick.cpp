#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>

#define device "/dev/input/js1"

int main()
{
    int n_axis = 0;
    int x;
    int n_bots = 0;
    int axis[10];
    char nome_joystick[80];
    char button[20];
    struct js_event joy;

    int js = open(device,O_RDONLY);

    ioctl( js, JSIOCGAXES, &n_axis );
    ioctl( js, JSIOCGBUTTONS, &n_bots );
    ioctl( js, JSIOCGNAME(80), &nome_joystick );

    //printf("%s %d %d\n",nome_joystick,n_bots,n_axis);

    fcntl( js, F_SETFL, O_NONBLOCK );

    while(1)
    {
        read(js, &joy, sizeof(struct js_event));

        switch (joy.type & ~JS_EVENT_INIT)
        {
            case JS_EVENT_AXIS:
                axis   [ joy.number ] = joy.value;
                break;
            case JS_EVENT_BUTTON:
                button [ joy.number ] = joy.value;
                break;
        }

            //printf("R: %6d  ", axis[3] );
        printf("%6d %6d %6d %6d",axis[0],axis[1],axis[2],axis[3]);
            
        //for( x=0 ; x<n_bots ; ++x )
            //printf("B%d: %d  ", x, button[x] );

        printf("  \n");
        //fflush(stdout);
        usleep(33332);

    }


    close( js );
    return 0;
}
