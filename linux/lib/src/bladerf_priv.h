/* This file is not part of the API and may be changed on a whim.
 * If you're interfacing with libbladerf, DO NOT use this file! */

#ifndef BLADERF_PRIV_H_
#define BLADERF_PRIV_H_

#ifdef USELIBUSB
    #ifdef _MSC_VER
        #include <libusb.h>
    #else
        #include <libusb-1.0/libusb.h>
    #endif /* _MSC_VER */
#endif /* USELIBUSB */


typedef enum {
    KERNEL,
    LIBUSB
} bladerf_driver ;

/* TODO Should there be a "big-lock" wrapped around accesses to a device */
struct bladerf {
    int fd;         /* File descriptor to associated driver device node */
    char *path;     /* Path of the opened fd */
    int speed;      /* The device's USB speed, 0 is HS, 1 is SS */
    struct bladerf_stats stats;

    int last_errno; /* Added for debugging. TODO Remove or integrate into
                     * error reporting? (I vote the latter -- Jon). */

    /* FIXME temporary workaround for not being able to read back sample rate */
    unsigned int last_tx_sample_rate;
    unsigned int last_rx_sample_rate;

    bladerf_driver driver ;

#ifdef USELIBUSB
    /* libusb interfacing */
    libusb_device           *usb_dev;
    libusb_device_handle    *usb_handle;
    libusb_context          *usb_context;
    int                     usb_errno;
#endif
};

#endif

