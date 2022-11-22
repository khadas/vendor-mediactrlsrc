#include <stdio.h>
#include "amlv4l2.h"

int main(void) {
    int video_fd;
    const char *media_device_name = "/dev/media0";

    video_fd = amlv4l2_open(media_device_name);
    printf("[client_test]video_fd: %d\n", video_fd);

    amlv4l2_notify_streamon();

    return 0;
}
