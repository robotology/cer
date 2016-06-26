#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>

#define CER_PATH "/dev/micif_dev"
#define FIFO_PATH "/tmp/"
#define FIFO_CH0_L "ch0_L"
#define FIFO_CH0_R "ch0_R"
#define FIFO_STEREO "ch0_Stereo"
#define CH_NUM 2
#define SAMP_BUF_SIZE 1024
#define STEREO 2
#define PSEUDO_STEREO STEREO

#define MAGIC_NUM 100
#define IOCTL_SAMPLERATE _IOW(MAGIC_NUM, 1, int )
#define IOCTL_SETBURST   _IOWR(MAGIC_NUM, 38, int )

int delete_fifo(const char *path)
{
    struct stat tmp;
    if (stat(path, &tmp) == 0)
    {
        if (unlink(path) < 0)
        {
        fprintf(stderr, "Unable to delete FIFO %s: %s\n", path, strerror(errno));
        return -1;
        }
    }
    return 0;
}

int handle_kill(int sig)
{
    delete_fifo(FIFO_PATH FIFO_CH0_L);
    delete_fifo(FIFO_PATH FIFO_CH0_R);
    delete_fifo(FIFO_PATH FIFO_STEREO);
    printf("CER Audio Mixer exiting\n");
    exit(0);
}

int create_fifo(const char *path)
{
    struct stat tmp;
    if (stat(path, &tmp) == 0)
    {
        if (unlink(path) < 0)
        {
            fprintf(stderr, "Unable to delete FIFO %s: %s\n", path, strerror(errno));
            return -1;
        }
    }
    if (mkfifo(path, 0666) < 0)
    {
        fprintf(stderr, "Error creating %s FIFO: %s\n", path, strerror(errno));
        return -1;
    }
    return 0;
}

int open_fifo(const char *path)
{
    return open(path, O_WRONLY | O_NONBLOCK);
}


int main()
{
    int l_fd = -1, r_fd = -1, s_fd = -1;
    int cer_fd = -1;
    int i;
    int32_t *l_val, *r_val, *s_val;
    int32_t buf[SAMP_BUF_SIZE * CH_NUM ] = { 0 };
    int32_t l_buf[PSEUDO_STEREO * SAMP_BUF_SIZE/CH_NUM ] = { 0 };
    int32_t r_buf[PSEUDO_STEREO * SAMP_BUF_SIZE/CH_NUM ] = { 0 };
    int32_t s_buf[STEREO * SAMP_BUF_SIZE/CH_NUM ] = { 0 };

    if (SAMP_BUF_SIZE%2)
    {
        printf("\nBe carefull, you are attempting to read an odd number of data? Not  STEREO???\n\n");
    }
    signal(SIGPIPE, SIG_IGN);
    signal(SIGTERM, (__sighandler_t) handle_kill);
    signal(SIGINT, (__sighandler_t) handle_kill);

    if (create_fifo(FIFO_PATH FIFO_CH0_L) < 0)
        return -1;
    if (create_fifo(FIFO_PATH FIFO_CH0_R) < 0)
        return -1;
    if (create_fifo(FIFO_PATH FIFO_STEREO) < 0)
        return -1;

    cer_fd = open(CER_PATH, O_RDONLY);
    if (cer_fd < 0)
    {
        fprintf(stderr, "Error opening %s: %s\n", CER_PATH, strerror(errno));
        return -1;
    }

    // set burst to 512
    ioctl(cer_fd,IOCTL_SETBURST,512);
    // Set sampling frequency to 12.8KHz
    ioctl(cer_fd,IOCTL_SAMPLERATE,60);
    while (1)
    {
        if (read(cer_fd, buf, CH_NUM * sizeof(int32_t) * SAMP_BUF_SIZE) < 0)
        {
            return -1;
        }

        l_val = l_buf;
        r_val = r_buf;
        s_val = s_buf;
        for (i = 0; i < SAMP_BUF_SIZE; i++)
        {
            if (i%2)
            {
                // Right sono i dispari
                *r_val = 0;
                r_val++;
                *r_val = buf[i*CH_NUM]<<8; r_val++;
                *s_val = buf[i*CH_NUM]<<8; s_val++;
            }
            else
            {
                // Left sono i pari
                *l_val = buf[i*CH_NUM]<<8; l_val++;
                *l_val = 0;
                l_val++;
                *s_val = buf[i*CH_NUM]<<8; s_val++;
            }
        }
        if (l_fd < 0)
            l_fd = open_fifo(FIFO_PATH FIFO_CH0_L);
            if (r_fd < 0)
        r_fd = open_fifo(FIFO_PATH FIFO_CH0_R);
        if (s_fd < 0)
            s_fd = open_fifo(FIFO_PATH FIFO_STEREO);


        if (l_fd)
        {
            if (write(l_fd, l_buf, sizeof(int32_t) * SAMP_BUF_SIZE ) < 0)
            {
                close(l_fd);
                l_fd = -1;
            }
        }
        if (r_fd)
        {
            if (write(r_fd, r_buf, sizeof(int32_t) * SAMP_BUF_SIZE ) < 0)
            {
            close(r_fd);
            r_fd = -1;
            }
        }

        if (s_fd > 0)
        {
            if (write(s_fd, s_buf, sizeof(int32_t) * SAMP_BUF_SIZE ) < 0)
            {
            close(s_fd);
            s_fd = -1;
            }
        }

    }
    return 0;
}
