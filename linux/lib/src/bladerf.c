#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <dirent.h>
#include <assert.h>
#include <time.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "bladeRF.h"        /* Driver interface */
#include "libbladeRF.h"     /* API */
#include "bladerf_priv.h"   /* Implementation-specific items ("private") */
#include "debug.h"

#ifndef BLADERF_DEV_DIR
#   define BLADERF_DEV_DIR "/dev/"
#endif

#ifndef BLADERF_DEV_PFX
#   define BLADERF_DEV_PFX  "bladerf"
#endif

#ifdef USELIBUSB
    #ifdef _MSC_VER
        #include <libusb.h>
    #else
        #include <libusb-1.0/libusb.h>
    #endif /* _MSC_VER */
#endif /* USELIBUSB */

/* TODO Check for truncation (e.g., odd # bytes)? */
static inline size_t bytes_to_c16_samples(size_t n_bytes)
{
    return n_bytes / (2 * sizeof(int16_t));
}

/* TODO Overflow check? */
static inline size_t c16_samples_to_bytes(size_t n_samples)
{
    return n_samples * 2 * sizeof(int16_t);
}

static inline size_t min_sz(size_t x, size_t y)
{
    return x < y ? x : y;
}

/*******************************************************************************
 * Device discovery & initialization/deinitialization
 ******************************************************************************/

/* Return 0 if dirent name matches that of what we expect for a bladerf dev */
static int bladerf_filter(const struct dirent *d)
{
    const size_t pfx_len = strlen(BLADERF_DEV_PFX);
    long int tmp;
    char *endptr;

    if (strlen(d->d_name) > pfx_len &&
        !strncmp(d->d_name, BLADERF_DEV_PFX, pfx_len)) {

        /* Is the remainder of the entry a valid (positive) integer? */
        tmp = strtol(&d->d_name[pfx_len], &endptr, 10);

        /* Nope */
        if (*endptr != '\0' || tmp < 0 ||
            (errno == ERANGE && (tmp == LONG_MAX || tmp == LONG_MIN)))
            return 0;

        /* Looks like a bladeRF by name... we'll check more later. */
        return 1;
    }

    return 0;
}

/* Helper routine for freeing dirent list from scandir() */
static inline void free_dirents(struct dirent **d, int n)
{
    if (d && n > 0 ) {
        while (n--) {
            free(d[n]);
            d[n] = NULL;
        }
        free(d);
        d = NULL;
    }
}

int _bladerf_init_device(struct bladerf *dev)
{
    /* Set the GPIO pins to enable the LMS and select the low band */
    gpio_write( dev, 0x51 );

    /* Set the internal LMS register to enable RX and TX */
    lms_spi_write( dev, 0x05, 0x3e );

    /* LMS FAQ: Improve TX spurious emission performance */
    lms_spi_write( dev, 0x47, 0x40 );

    /* LMS FAQ: Improve ADC performance */
    lms_spi_write( dev, 0x59, 0x29 );

    /* LMS FAQ: Common mode voltage for ADC */
    lms_spi_write( dev, 0x64, 0x36 );

    /* LMS FAQ: Higher LNA Gain */
    lms_spi_write( dev, 0x79, 0x37 );

    /* LMS Programming Guide suggests I_OFF UP to be 30uA */
    lms_spi_write( dev, 0x17, 0xe3 );
    lms_spi_write( dev, 0x27, 0xe3 );

    /* TODO: Read this return from the SPI calls */
    return 0;
}

/* Open and if a non-NULL bladerf_devinfo ptr is provided, attempt to verify
 * that the device we opened is a bladeRF via a info calls.
 * (Does not fill out devinfo's path) */
struct bladerf * _bladerf_open_info(const char *dev_path,
                                    struct bladerf_devinfo *i)
{
    struct bladerf *ret;
    unsigned int speed;
    char *tok;

    ret = malloc(sizeof(*ret));
    if (!ret)
        return NULL;

    fprintf( stderr, "path: %s\n", dev_path );
    ret->last_errno = 0;
    ret->last_tx_sample_rate = 0;
    ret->last_rx_sample_rate = 0;
    ret->driver = KERNEL;
#ifdef USELIBUSB
    ret->usb_handle = NULL;
    ret->usb_context = NULL;
    ret->usb_errno = 0;
#endif

    /* TODO -- spit out error/warning message to assist in debugging
     * device node permissions issues?
     */

#ifdef USELIBUSB
    /* Check if it's a libusb path in the form libusb-n */
    if( strstr(dev_path,"libusb-") != NULL ) {
        int n, status;
        errno = 0 ;

        /* Parse out the index and check for errors*/
        n = strtol(dev_path+7, NULL, 10);
        if( errno != 0 && index == 0 ) {
            ret->last_errno = errno;
            goto bladerf_open__err;
        }

        /* Open context and set driver */
        ret->driver = LIBUSB ;

        /* Open the nth device */
        status = _bladerf_open_libusb(ret, n);

        /* If we have issues ... */
        if( status < 0 ) {
            ret->usb_errno = status;
            goto bladerf_open__err;
        }

        /* Determine the device's USB speed */
        /* TODO: Match libusb's speed rating so we don't have this disparity */
        ret->speed = libusb_get_device_speed(ret->usb_dev) - 3;
    } else
#endif
    {
        /* Kernel driver path */
        ret->driver = KERNEL;
        if ((ret->fd = open(dev_path, O_RDWR)) < 0) {
            ret->last_errno = errno;
            goto bladerf_open__err;
        }

        /* Determine the device's USB speed */
        if (_bladerf_ioctl(ret->fd, BLADE_GET_SPEED, &speed, sizeof(speed)))
            goto bladerf_open__err;
        ret->speed = speed;
    }

    /* Successfully opened, so save off the path */
    if( (ret->path = strdup(dev_path)) == NULL) {
        /* Not so successful string duplication */
        ret->last_errno = errno;
        goto bladerf_open__err;
    }

    fprintf( stderr, "ret->path: %s %p\n", ret->path, ret->path );

    /* TODO -- spit our errors/warning here depending on library verbosity? */
    if (ret->driver == KERNEL && i) {
        fprintf( stderr, "Getting more info\n" ) ;
        if (bladerf_get_serial(ret, &i->serial) < 0) {
            goto bladerf_open__err;
        }

        i->fpga_configured = bladerf_is_fpga_configured(ret);
        if (i->fpga_configured < 0) {
            goto bladerf_open__err;
        }

        if (bladerf_get_fw_version(ret, &i->fw_ver_maj, &i->fw_ver_min) < 0) {
            goto bladerf_open__err;
        }

        if (bladerf_get_fpga_version(ret,
                                     &i->fpga_ver_maj, &i->fpga_ver_min) < 0) {
            goto bladerf_open__err;
        }
    }

    return ret;

bladerf_open__err:
    fprintf( stderr, "We just free'd up ret :(\n" );
    free(ret);
    ret = NULL;
    return NULL;
}

#ifdef USELIBUSB
int _libusb_device_is_bladerf( libusb_device *dev )
{
    int err ;
    int rv = 0 ;
    struct libusb_device_descriptor desc;

    err = libusb_get_device_descriptor( dev, &desc );
    if( err ) {
        fprintf( stderr, "%s(%d): Couldn't open libusb device - %s\n", __FILE__, __LINE__, libusb_error_name(err) ) ;
    } else {
        if( desc.idVendor == USB_NUAND_VENDOR_ID && desc.idProduct == USB_NUAND_BLADERF_PRODUCT_ID ) {
            rv = 1;
        }
    }

    return rv;

}

ssize_t _libusb_get_bladerf_count()
{
    libusb_context *context;
    ssize_t usb_devs ;
    int init = libusb_init(&context);
    libusb_device **list ;
    int i;
    ssize_t num_devices = 0;
    usb_devs = libusb_get_device_list( NULL, &list );

    /* Foreach of the usb devices ... */
    for(i = 0; i < usb_devs ; i++ ) {
        if( _libusb_device_is_bladerf(list[i]) ) {
            /* Increment device count */
            num_devices++;
        }
    }
    libusb_free_device_list( list, 1 );
    libusb_exit(context);
    return num_devices;
}

int _bladerf_open_libusb(struct bladerf *dev, int index)
{
    int status, i, n;
    ssize_t count;
    uint8_t res;
    libusb_device **list;
    status = libusb_init(&dev->usb_context);
    if( status ) {
        return status ;
    }
    count = libusb_get_device_list(NULL, &list);
    for( i = 0, n = 0 ; i < count ; i++ ) {
        if( _libusb_device_is_bladerf(list[i]) ) {
            if( n == index ) {
                dev->usb_dev = list[i];
                status = libusb_open( list[i], &dev->usb_handle );
                libusb_claim_interface(dev->usb_handle, 0);
                libusb_claim_interface(dev->usb_handle, 1);
                libusb_claim_interface(dev->usb_handle, 2);
                libusb_control_transfer(
                    dev->usb_handle,
                    LIBUSB_RECIPIENT_DEVICE | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_ENDPOINT_IN,
                    10,
                    0,
                    0,
                    &res,
                    1,
                    0
                ) ;
                break;
            }
            n++;
        }
    }
    if( dev->usb_dev == NULL ) {
        status = LIBUSB_ERROR_NO_DEVICE;
    }
    libusb_free_device_list( list, 1 );
    return status ;
}

#endif

ssize_t bladerf_get_device_list(struct bladerf_devinfo **devices)
{
    struct bladerf_devinfo *ret;
    ssize_t num_devices;
    struct dirent **matches;
    int num_matches, i;
    struct bladerf *dev;
    char *dev_path;

    ret = NULL;
    num_devices = 0;

    /* Kernel method */
    num_matches = scandir(BLADERF_DEV_DIR, &matches, bladerf_filter, alphasort);
    if (num_matches > 0) {

        ret = malloc(num_matches * sizeof(*ret));
        if (!ret) {
            num_devices = BLADERF_ERR_MEM;
            goto bladerf_get_device_list_out;
        }

        for (i = 0; i < num_matches; i++) {
            dev_path = malloc(strlen(BLADERF_DEV_DIR) +
                                strlen(matches[i]->d_name) + 1);

            if (dev_path) {
                strcpy(dev_path, BLADERF_DEV_DIR);
                strcat(dev_path, matches[i]->d_name);

                dev = _bladerf_open_info(dev_path, &ret[num_devices]);

                if (dev) {
                    ret[num_devices++].path = dev_path;
                    bladerf_close(dev);
                } else
                    free(dev_path);
                    dev_path = NULL;
            } else {
                num_devices = BLADERF_ERR_MEM;
                goto bladerf_get_device_list_out;
            }
        }
    } else {
        /* libusb method */
        fprintf( stderr, "Trying to open bladeRF using libusb\n" );

        /* Find all the bladeRF's connected */
        num_devices = _libusb_get_bladerf_count() ;
        fprintf( stderr, "Found %zd devices\n", num_devices );
        if( num_devices > 0 ) {

            ret = malloc(sizeof(*ret) * num_devices);

            /* Foreach of the devices ... */
            for(i = 0; i < num_devices ; i++ ) {
                /* Save off the path */
                dev_path = malloc(strlen("libusb-###") + 1) ;

                if( dev_path ) {
                    snprintf( dev_path, 11, "libusb-%3.3d", i );
                    fprintf( stderr, "dev_path: %s\n", dev_path );

                    /* Populate information */
                    dev = _bladerf_open_info( dev_path, &ret[i] );
                    fprintf( stderr, "from the info: %s\n", ret[i].path ) ;

                    /* Free out temporary storage */
                    free(dev_path);
                    dev_path = NULL;
                } else {
                    num_devices = BLADERF_ERR_MEM;
                    goto bladerf_get_device_list_out;
                }
            }
            fprintf( stderr, "Done with libusb open\n" ) ;
        }
    }


bladerf_get_device_list_out:
    *devices = ret;
    free_dirents(matches, num_matches);
    return num_devices;
}

void bladerf_free_device_list(struct bladerf_devinfo *devices, size_t n)
{
    size_t i;

    if (devices) {
        for (i = 0; i < n; i++) {
            fprintf( stderr, "Freeing: %s\n", devices[i].path ) ;
            free(devices[i].path);
            devices[i].path = NULL;
        }
        free(devices);
        devices = NULL;
    }
}

char * bladerf_dev_path(struct bladerf *dev)
{
    if( dev ) {
        return dev->path;
    } else {
        return NULL;
    }
}

struct bladerf * bladerf_open(const char *dev_path)
{
    struct bladerf_devinfo i;

    /* Use open with devinfo check to verify that this is actually a bladeRF */
    return _bladerf_open_info(dev_path, &i);
}

struct bladerf * bladerf_open_any()
{
    struct bladerf *ret = NULL;
    struct bladerf_devinfo *devices;
    ssize_t n_devices;

    n_devices = bladerf_get_device_list(&devices);
    fprintf( stderr, "any found: %zd devices\n", n_devices ) ;
    if (n_devices > 0) {
        ret = bladerf_open(devices[0].path);
        bladerf_free_device_list(devices, n_devices);
    }

    return ret;
}


void bladerf_close(struct bladerf *dev)
{
    if (dev) {
        if( dev->driver == KERNEL ) {
            close(dev->fd);
        } else {
            libusb_close(dev->usb_handle);
            libusb_exit(dev->usb_context);
        }
        free(dev->path);
        dev->path = NULL;
        free(dev);
        dev = NULL;
    }
}

int bladerf_enable_module(struct bladerf *dev,
                            bladerf_module m, bool enable)
{
    int status = BLADERF_ERR_UNEXPECTED;
    uint32_t gpio_reg;

    status = gpio_read(dev, &gpio_reg);

    if (status == 0) {
        switch (m) {
            case TX:
                if (enable) {
                    gpio_reg |= BLADERF_GPIO_LMS_TX_ENABLE;
                } else {
                    gpio_reg &= ~BLADERF_GPIO_LMS_TX_ENABLE;
                }
                break;
            case RX:
                if (enable) {
                    gpio_reg |= BLADERF_GPIO_LMS_RX_ENABLE;
                } else {
                    gpio_reg &= ~BLADERF_GPIO_LMS_RX_ENABLE;
                }
                break;
        }

        status = gpio_write(dev, gpio_reg);
    }

    return status;
}

int bladerf_set_loopback(struct bladerf *dev, bladerf_loopback l)
{
    lms_loopback_enable( dev, l );
    return 0;
}

int bladerf_set_sample_rate(struct bladerf *dev, bladerf_module module, unsigned int rate, unsigned int *actual)
{
    int ret = -1;
    /* TODO: Use module to pick the correct clock output to change */
    if( module == TX ) {
        ret = si5338_set_tx_freq(dev, rate<<1);
        dev->last_tx_sample_rate = rate;
    } else {
        ret = si5338_set_rx_freq(dev, rate<<1);
        dev->last_rx_sample_rate = rate;
    }
    *actual = rate;
    return ret;
}

int bladerf_set_rational_sample_rate(struct bladerf *dev, bladerf_module module, unsigned int integer, unsigned int num, unsigned int denom)
{
    /* TODO: Program the Si5338 to be 2x the desired sample rate */
    return 0;
}

int bladerf_get_sample_rate( struct bladerf *dev, bladerf_module module, unsigned int *rate)
{
    /* TODO Reconstruct samplerare from Si5338 readback */
    if (module == RX) {
        *rate = dev->last_rx_sample_rate;
    } else if (module == TX) {
        *rate = dev->last_tx_sample_rate;
    } else {
        return BLADERF_ERR_INVAL;
    }

    return 0;
}

int bladerf_get_rational_sample_rate(struct bladerf *dev, bladerf_module module, unsigned int integer, unsigned int num, unsigned int denom)
{
    /* TODO: Read the Si5338 and figure out the sample rate */
    return 0;
}

int bladerf_set_txvga2(struct bladerf *dev, int gain)
{
    if( gain > 25 ) {
        fprintf( stderr, "%s: %d being clamped to 25dB\n", __FUNCTION__, gain );
        gain = 25;
    }
    if( gain < 0 ) {
        fprintf( stderr, "%s: %d being clamped to 0dB\n", __FUNCTION__, gain );
        gain = 0;
    }
    /* TODO: Make return values for lms call and return it for failure */
    lms_txvga2_set_gain( dev, gain );
    return 0;
}

int bladerf_get_txvga2(struct bladerf *dev, int *gain)
{
    uint8_t gain_u8;
    /* TODO: Make return values for lms call and return it for failure */
    lms_txvga2_get_gain( dev, &gain_u8 );
    *gain = gain_u8;
    return 0;
}

int bladerf_set_txvga1(struct bladerf *dev, int gain)
{
    if( gain < -35 ) {
        fprintf( stderr, "%s: %d being clamped to -35dB\n", __FUNCTION__, gain );
        gain = -35;
    }
    if( gain > -4 ) {
        fprintf( stderr, "%s: %d being clamped to -4dB\n", __FUNCTION__, gain );
        gain = -4;
    }
    /* TODO: Make return values for lms call and return it for failure */
    lms_txvga1_set_gain( dev, gain );
    return 0;
}

int bladerf_get_txvga1(struct bladerf *dev, int *gain)
{
    *gain = 0;
    /* TODO: Make return values for lms call and return it for failure */
    lms_txvga1_get_gain( dev, (int8_t *)gain );
    *gain |= 0xffffff00 ;
    return 0;
}

int bladerf_set_lna_gain(struct bladerf *dev, bladerf_lna_gain gain)
{
    /* TODO: Make return values for lms call and return it for failure */
    lms_lna_set_gain( dev, gain );
    return 0;
}

int bladerf_get_lna_gain(struct bladerf *dev, bladerf_lna_gain *gain)
{
    /* TODO: Make return values for lms call and return it for failure */
    lms_lna_get_gain( dev, gain );
    return 0;
}

int bladerf_set_rxvga1(struct bladerf *dev, int gain)
{
    int r;
    /* TODO: Make return values for lms call and return it for failure */
    for (r=2; (r * r) < ((gain - 5) << 9); r++);
    lms_rxvga1_set_gain( dev, r );
    return 0;
}

int bladerf_get_rxvga1(struct bladerf *dev, int *gain)
{
    uint8_t gain_u8;
    /* TODO: Make return values for lms call and return it for failure */
    lms_rxvga1_get_gain( dev, &gain_u8 );
    *gain = 5 + (((int)gain_u8 * (int)gain_u8) >> 9);
    return 0;
}

int bladerf_set_rxvga2(struct bladerf *dev, int gain)
{
    /* TODO: Make return values for lms call and return it for failure */
    /* Raw value to write which is 3dB per unit */
    lms_rxvga2_set_gain( dev, (uint8_t)gain/3 );
    return 0;
}

int bladerf_get_rxvga2(struct bladerf *dev, int *gain)
{
    uint8_t gain_u8;
    /* TODO: Make return values for lms call and return it for failure */
    lms_rxvga2_get_gain( dev, &gain_u8 );
    /* Value returned from LMS lib is just the raw value, which is 3dB/unit */
    *gain = gain_u8 * 3;
    return 0;
}

int bladerf_set_bandwidth(struct bladerf *dev, bladerf_module module,
                            unsigned int bandwidth,
                            unsigned int *actual)
{
    /* TODO: Make return values for lms call and return it for failure */
    lms_bw_t bw = lms_uint2bw(bandwidth);
    lms_lpf_enable( dev, module, bw );
    *actual = lms_bw2uint(bw);
    return 0;
}

int bladerf_get_bandwidth(struct bladerf *dev, bladerf_module module,
                            unsigned int *bandwidth )
{
    /* TODO: Make return values for lms call and return it for failure */
    lms_bw_t bw = lms_get_bandwidth( dev, module );
    *bandwidth = lms_bw2uint(bw);
    return 0;
}

int bladerf_select_band(struct bladerf *dev, bladerf_module module,
                        unsigned int frequency)
{
    uint32_t gpio ;
    uint32_t band = 0;
    if( frequency >= 1500000000 ) {
        band = 1; // High Band selection
        lms_lna_select(dev, LNA_2);
    } else {
        band = 2; // Low Band selection
        lms_lna_select(dev, LNA_1);
    }
    gpio_read(dev, &gpio);
    gpio &= ~(module == TX ? (3<<3) : (3<<5));
    gpio |= (module == TX ? (band<<3) : (band<<5));
    gpio_write(dev, gpio);
    return 0;
}

int bladerf_set_frequency(struct bladerf *dev,
                            bladerf_module module, unsigned int frequency)
{
    /* TODO: Make return values for lms call and return it for failure */
    lms_set_frequency( dev, module, frequency );
    bladerf_select_band( dev, module, frequency );
    return 0;
}

int bladerf_get_frequency(struct bladerf *dev,
                            bladerf_module module, unsigned int *frequency)
{
    /* TODO: Make return values for lms call and return it for failure */
    struct lms_freq f;
    int rv = 0;
    lms_get_frequency( dev, module, &f );
    if( f.x == 0 ) {
        *frequency = 0 ;
        rv = BLADERF_ERR_INVAL;
    } else {
        *frequency = lms_frequency_to_hz(&f);
    }
    return rv;
}
/*******************************************************************************
 * Data transmission and reception
 *
 * Note that:
 *  For c16:  1 sample   =   1 i,q pair   =   2 int16_t's  =  4 bytes
 ******************************************************************************/

ssize_t bladerf_send_c12(struct bladerf *dev, int16_t *samples, size_t n)
{
    return 0;
}

/* TODO Fail out if n > ssize_t max, as we can't return that. */
ssize_t bladerf_send_c16(struct bladerf *dev, int16_t *samples, size_t n)
{
    ssize_t i, ret = 0;
    size_t bytes_written = 0;
    const size_t bytes_total = c16_samples_to_bytes(n);

    while (bytes_written < bytes_total) {

        i = write(dev->fd, samples + bytes_written, bytes_total - bytes_written);

        if (i < 0 && errno != EINTR) {
            dev->last_errno = errno;
            bytes_written = BLADERF_ERR_IO;
            dbg_printf("Failed to write with errno=%d: %s\n",
                    dev->last_errno, strerror(dev->last_errno));
            break;
        } else if (i > 0) {
            bytes_written += i;
        } else {
            dbg_printf("\nInterrupted in bladerf_send_c16 (%zd/%zd\n",
                       bytes_written, bytes_total);
        }
    }

    if (ret < 0) {
        return ret;
    } else {
        return bytes_to_c16_samples(bytes_written);
    }
}

/* TODO Fail out if n > ssize_t max, as we can't return that */
ssize_t bladerf_read_c16(struct bladerf *dev,
                            int16_t *samples, size_t n)
{
    ssize_t i, ret = 0;
    size_t bytes_read = 0;
    const size_t bytes_total = c16_samples_to_bytes(n);

    while (bytes_read < bytes_total) {

        i = read(dev->fd, samples + bytes_read, bytes_total - bytes_read);

        if (i < 0 && errno != EINTR) {
            dev->last_errno = errno;
            ret = BLADERF_ERR_IO;
            dbg_printf("Read failed with errno=%d: %s\n",
                        dev->last_errno, strerror(dev->last_errno));
            break;
        } else if (i > 0) {
            bytes_read += i;
        } else {
            dbg_printf("\nInterrupted in bladerf_read_c16 (%zd/%zd\n",
                       bytes_read, bytes_total);
        }
    }

    if (ret < 0) {
        return ret;
    } else {
        return bytes_to_c16_samples(bytes_read);
    }
}

/*******************************************************************************
 * Device info
 ******************************************************************************/

/* TODO - Devices do not currently support serials */
int bladerf_get_serial(struct bladerf *dev, uint64_t *serial)
{
    assert(dev && serial);

    *serial = 0;
    return 0;
}

#ifdef USELIBUSB
int _bladerf_read_peripheral(struct bladerf *dev, uint8_t uart_dev, struct uart_cmd *cmd )
{
    uint8_t buf[16];
    int ret, res;
    int transferred;

    buf[0] = UART_PKT_MAGIC ;
    buf[1] = UART_PKT_MODE_DIR_READ | uart_dev | 0x01 ;
    buf[2] = cmd->addr ;
    buf[3] = 0xff ;

    res = libusb_bulk_transfer(dev->usb_handle, 0x02, buf, 16, &transferred, 100);
    if( res ) {
        dev->usb_errno = res ;
        return res ;
    }
    transferred = 0 ;
    while( res == 0 && transferred != 16 ) {
        res = libusb_bulk_transfer(dev->usb_handle, 0x82, buf, 16, &transferred, 100);
        if( res ) {
            dev->usb_errno = res ;
        }
    }

    cmd->data = buf[3] ;
    return res;
}

int _bladerf_write_peripheral(struct bladerf *dev, uint8_t uart_dev, struct uart_cmd *cmd )
{
    uint8_t buf[16];
    int ret, res;
    int transferred;

    buf[0] = UART_PKT_MAGIC ;
    buf[1] = UART_PKT_MODE_DIR_WRITE | uart_dev | 0x01 ;
    buf[2] = cmd->addr ;
    buf[3] = cmd->data ;

    res = libusb_bulk_transfer(dev->usb_handle, 0x02, buf, 16, &transferred, 100);
    if( res ) {
        dev->usb_errno = res ;
        return res ;
    }
    transferred = 0 ;
    while( res && transferred != 16 ) {
        res = libusb_bulk_transfer(dev->usb_handle, 0x82, buf, 16, &transferred, 100);
        if( res ) {
            dev->usb_errno = res ;
        }
    }

    return res ;
}
#endif /* USELIBUSB */

int _bladerf_ioctl(struct bladerf *dev, int cmd, void *data, size_t size)
{
    int status ;
    uint8_t peripheral = 0xff ;
    struct uart_cmd *peripheral_cmd ;

    switch(dev->driver) {
        /* Kernel is just a pass through */
        case KERNEL:
            status = ioctl( dev->fd, cmd, data );
            break;
#ifdef USELIBUSB
        /* libusb requires some translation */
        case LIBUSB:
            /* Translate cmd into usb_cmd */
            switch(cmd) {
                case BLADE_QUERY_VERSION    : cmd = BLADE_USB_CMD_QUERY_VERSION; break;
                case BLADE_QUERY_FPGA_STATUS: cmd = BLADE_USB_CMD_QUERY_FPGA_STATUS; break;
                case BLADE_BEGIN_PROG       : cmd = BLADE_USB_CMD_BEGIN_PROG; break;
                case BLADE_END_PROG         : cmd = BLADE_USB_CMD_END_PROG; break;
                case BLADE_RF_RX            : cmd = BLADE_USB_CMD_RF_RX; break;
                case BLADE_RF_TX            : cmd = BLADE_USB_CMD_RF_TX; break;
                case BLADE_CHECK_PROG       :
                case BLADE_FLASH_READ       :
                case BLADE_FLASH_WRITE      :
                case BLADE_OTP_READ         :
                case BLADE_UPGRADE_FW       :
                case BLADE_CAL              :
                case BLADE_OTP              : fprintf( stderr, "Random stuff not implemented\n" ); break;
                /* UART Reading Commands */
                case BLADE_LMS_READ         : cmd = BLADE_USB_CMD_UART_READ ; peripheral = UART_PKT_DEV_LMS ; break ;
                case BLADE_SI5338_READ      : cmd = BLADE_USB_CMD_UART_READ ; peripheral = UART_PKT_DEV_SI5338 ; break ;
                case BLADE_GPIO_READ        : cmd = BLADE_USB_CMD_UART_READ ; peripheral = UART_PKT_DEV_GPIO ; break;
                /* UART Writing Commands */
                case BLADE_LMS_WRITE        : cmd = BLADE_USB_CMD_UART_WRITE ; peripheral = UART_PKT_DEV_LMS ; break ;
                case BLADE_VCTCXO_WRITE     : cmd = BLADE_USB_CMD_UART_WRITE ; peripheral = UART_PKT_DEV_VCTCXO ; break ;
                case BLADE_SI5338_WRITE     : cmd = BLADE_USB_CMD_UART_WRITE ; peripheral = UART_PKT_DEV_SI5338 ; break ;
                case BLADE_GPIO_WRITE       : cmd = BLADE_USB_CMD_UART_WRITE ; peripheral = UART_PKT_DEV_GPIO ; break ;
                default                     : fprintf( stderr, "Something undiscovered?\n" ); break ;
            }
            /* Perform the operation */
            if( cmd == BLADE_USB_CMD_UART_READ ) {
                peripheral_cmd = (struct uart_cmd *)data ;
                status = _bladerf_read_peripheral( dev, peripheral, peripheral_cmd );
            } else if( cmd == BLADE_USB_CMD_UART_WRITE ) {
                peripheral_cmd = (struct uart_cmd *)data ;
                status = _bladerf_write_peripheral( dev, peripheral, peripheral_cmd );
            } else {
               /* TODO: The other IO things */
               //fprintf( stderr, "ioctl command: %d\n", cmd ) ;
               status = libusb_control_transfer(
                            dev->usb_handle,
                            LIBUSB_RECIPIENT_INTERFACE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN,
                            cmd,
                            0,
                            0,
                            data,
                            size,
                            1000
                        ) ;
                if( status < 0 ) {
                    fprintf( stderr, "ioctl error: %s\n", libusb_error_name(status) );
                    dev->usb_errno = status ;
                } else {
                    // fprintf( stderr, "transferred %d bytes\n", status );
                    status = 0;
                }
            }
            break;
#endif
    }
    return status ;
}

int bladerf_is_fpga_configured(struct bladerf *dev)
{
    int status;
    int configured;
    int result;

    assert(dev);

    status = _bladerf_ioctl(dev, BLADE_QUERY_FPGA_STATUS, &configured, sizeof(configured));
    if (status || configured < 0 || configured > 1) {
        configured = BLADERF_ERR_IO;
    }
    //fprintf( stderr, "FPGA Configuration: %d\n", configured ) ;
    return configured;
}

/* TODO Not yet supported */
int bladerf_get_fpga_version(struct bladerf *dev,
                                unsigned int *major, unsigned int *minor)
{
    assert(dev && major && minor);

    *major = 0;
    *minor = 0;

    return 0;
}

int bladerf_get_fw_version(struct bladerf *dev,
                            unsigned int *major, unsigned int *minor)
{
    int status;
    struct bladeRF_version ver;

    assert(dev && major && minor);

    status = _bladerf_ioctl(dev, BLADE_QUERY_VERSION, &ver, sizeof(ver));
    if (!status) {
        *major = ver.major;
        *minor = ver.minor;
        return 0;
    }

    /* TODO return more appropriate error code based upon errno */
    return BLADERF_ERR_IO;
}

/*------------------------------------------------------------------------------
 * Misc.
 *----------------------------------------------------------------------------*/

const char * bladerf_strerror(int error)
{
    switch (error) {
        case BLADERF_ERR_UNEXPECTED:
            return "An unexpected error occurred";
        case BLADERF_ERR_RANGE:
            return "Provided parameter was out of the allowable range";
        case BLADERF_ERR_INVAL:
            return "Invalid operation or parameter";
        case BLADERF_ERR_MEM:
            return "A memory allocation error occurred";
        case BLADERF_ERR_IO:
            return "File or device I/O failure";
        case BLADERF_ERR_TIMEOUT:
            return "Operation timed out";
        case 0:
            return "Success";
        default:
            return "Unknown error code";
    }
}

/*------------------------------------------------------------------------------
 * Device programming
 *----------------------------------------------------------------------------*/

int bladerf_flash_firmware(struct bladerf *dev, const char *firmware)
{
    int fw_fd, upgrade_status, ret;
    struct stat fw_stat;
    struct bladeRF_firmware fw_param;
    ssize_t n_read, read_status;

    assert(dev && firmware);

    fw_fd = open(firmware, O_RDONLY);
    if (fw_fd < 0) {
        dbg_printf("Failed to open firmware file: %s\n", strerror(errno));
        return BLADERF_ERR_IO;
    }

    if (fstat(fw_fd, &fw_stat) < 0) {
        dbg_printf("Failed to stat firmware file: %s\n", strerror(errno));
        close(fw_fd);
        return BLADERF_ERR_IO;
    }

    fw_param.len = fw_stat.st_size;

    /* Quick sanity check: We know the firmware file is roughly 100K
     * Env var is a quick opt-out of this check - can it ever get this large?
     *
     * TODO: Query max flash size for upper bound?
     */
    if (!getenv("BLADERF_SKIP_FW_SIZE_CHECK") &&
            (fw_param.len < (50 * 1024) || fw_param.len > (1 * 1024 * 1024))) {
        dbg_printf("Detected potentially invalid firmware file. Aborting!\n");
        close(fw_fd);
        return BLADERF_ERR_INVAL;
    }

    fw_param.ptr = malloc(fw_param.len);
    if (!fw_param.ptr) {
        dbg_printf("Failed to allocate firmware buffer: %s\n", strerror(errno));
        close(fw_fd);
        return BLADERF_ERR_MEM;
    }

    n_read = 0;
    do {
        read_status = read(fw_fd, fw_param.ptr + n_read, fw_param.len - n_read);
        if (read_status < 0) {
            dbg_printf("Failed to read firmware file: %s\n", strerror(errno));
            free(fw_param.ptr);
            fw_param.ptr = NULL;
            close(fw_fd);
            return BLADERF_ERR_IO;
        } else {
            n_read += read_status;
        }
    } while(n_read < fw_param.len);
    close(fw_fd);

    /* Bug catcher */
    assert(n_read == fw_param.len);

    ret = 0;
    upgrade_status = _bladerf_ioctl(dev, BLADE_UPGRADE_FW, &fw_param, sizeof(fw_param));
    if (upgrade_status < 0) {
        dbg_printf("Firmware upgrade failed: %s\n", strerror(errno));
        ret = BLADERF_ERR_UNEXPECTED;
    }

    free(fw_param.ptr);
    fw_param.ptr = NULL;
    return ret;
}

static bool time_past(struct timeval ref, struct timeval now) {
    if (now.tv_sec > ref.tv_sec)
        return true;

    if (now.tv_sec == ref.tv_sec && now.tv_usec > ref.tv_usec)
        return true;

    return false;
}

#define STACK_BUFFER_SZ 1024
int bladerf_load_fpga(struct bladerf *dev, const char *fpga)
{
    int ret, fpga_status, fpga_fd;
    ssize_t nread, written, write_tmp;
    size_t bytes;
    struct stat stat;
    char buf[STACK_BUFFER_SZ];
    struct timeval end_time, curr_time;
    bool timed_out;
    int res;
    int transferred;
    char *newbuf ;
    struct libusb_config_descriptor *conf ;
    int i ;

    assert(dev && fpga);

    /* TODO Check FPGA on the board versus size of image */

    if (_bladerf_ioctl(dev, BLADE_QUERY_FPGA_STATUS, &fpga_status, sizeof(fpga_status)) < 0) {
        dbg_printf("ioctl(BLADE_QUERY_FPGA_STATUS) failed: %s\n",
                    strerror(errno));
        return BLADERF_ERR_UNEXPECTED;
    }

    /* FPGA is already programmed */
    if (fpga_status) {
        fprintf( stderr, "FPGA aleady loaded - reloading!\n" );
    }

    fpga_fd = open(fpga, 0);
    if (fpga_fd < 0) {
        dbg_printf("Failed to open device (%s): %s\n", fpga, strerror(errno));
        return BLADERF_ERR_IO;
    }

    if (fstat(fpga_fd, &stat) < 0) {
        dbg_printf("Failed to stat fpga file (%s): %s\n", fpga, strerror(errno));
        close(fpga_fd);
        return BLADERF_ERR_IO;
    }

    fprintf( stderr, "Starting to program the FPGA\n" ) ;
#ifdef USELIBUSB
    if( dev->driver == LIBUSB ) {
        res = libusb_claim_interface(dev->usb_handle,1);
        if( res ) {
            fprintf( stderr, "error claiming interface: %s\n", libusb_error_name(res) );
        }
        res = libusb_set_interface_alt_setting(dev->usb_handle,1,0);
        if( res ) {
            fprintf( stderr, "error setting alt interface: %s\n", libusb_error_name(res) );
        }
        fprintf( stderr, "Claimed the interfaces\n" ) ;
    }
#endif
    if (_bladerf_ioctl(dev, BLADE_BEGIN_PROG, &fpga_status, sizeof(fpga_status))) {
        fprintf(stderr, "ioctl(BLADE_BEGIN_PROG) failed: %s\n", strerror(errno));
        close(fpga_fd);
        return BLADERF_ERR_UNEXPECTED;
    }

    newbuf = (uint8_t *)malloc(stat.st_size) ;
    bytes = read(fpga_fd, newbuf, stat.st_size);

    res = libusb_get_active_config_descriptor(dev->usb_dev, &conf) ;
    for(i = 0 ; i < conf->interface[0].altsetting[0].bNumEndpoints; i++ ) {
        fprintf( stderr, "EP: %x\n", conf->interface[1].altsetting[0].endpoint[i].bEndpointAddress ) ;
    }

    while( bytes > 0 ) {
        res = libusb_bulk_transfer( dev->usb_handle, 0x02, buf, bytes >= 4096 ? 4096 : bytes, &transferred, 1000 );
        fprintf( stderr, "res: %s transferred: %d\n", libusb_error_name(res), transferred );
        bytes -= transferred ;
        newbuf += transferred ;
    }

    while(true) {
        int r = bladerf_is_fpga_configured(dev);
        if(r) {
            break ;
        }
    }
    bytes = 0 ;
    free(newbuf) ;

//    for (bytes = stat.st_size; bytes;) {
//        nread = read(fpga_fd, buf, min_sz(STACK_BUFFER_SZ, bytes));
//
//        written = 0;
//        do {
//            if( dev->driver == KERNEL ) {
//                write_tmp = write(dev->fd, buf + written, nread - written);
//            }
//#ifdef USELIBUSB
//            else {
//                fprintf( stderr, "Writing %zd\n", nread-written );
//                res = libusb_bulk_transfer( dev->usb_handle, 0x02, buf + written, nread - written, &transferred, 0 );
//                fprintf( stderr, "  res: %d write_tmp: %zd\n", res, write_tmp );
//                write_tmp = transferred ;
//            }
//#endif
//            if (write_tmp < 0) {
//                /* Failing out...at least attempt to "finish" programming */
//                _bladerf_ioctl(dev, BLADE_END_PROG, &ret, sizeof(fpga_status));
//                fprintf(stderr, "Write failure: %s\n", strerror(errno));
//                close(fpga_fd);
//                return BLADERF_ERR_IO;
//            } else {
//                written += write_tmp;
//            }
//        } while(written < nread);
//
//        bytes -= nread;
//
//        /* FIXME? Perhaps it would be better if the driver blocked on the
//         * write call, rather than sleeping in userspace? */
//        usleep(4000);
//    }
    close(fpga_fd);

    /* Debug mode bug catcher */
    assert(bytes == 0);

    /* Time out within 1 second */
    gettimeofday(&end_time, NULL);
    end_time.tv_sec++;

    ret = 0;
    do {
        if (_bladerf_ioctl(dev, BLADE_QUERY_FPGA_STATUS, &fpga_status, sizeof(fpga_status)) < 0) {
            fprintf(stderr, "Failed to query FPGA status: %s\n", strerror(errno));
            ret = BLADERF_ERR_UNEXPECTED;
        }
        gettimeofday(&curr_time, NULL);
        timed_out = time_past(end_time, curr_time);
    } while(!fpga_status && !timed_out && !ret);

    if (_bladerf_ioctl(dev, BLADE_END_PROG, &fpga_status, sizeof(fpga_status))) {
        dbg_printf("Failed to end programming procedure: %s\n",
                strerror(errno));

        /* Don't clobber a previous error */
        if (!ret)
            ret = BLADERF_ERR_UNEXPECTED;
    }

    /* Now that the FPGA is loaded, initialize the device */
    fprintf( stderr, "Initializing device\n" );
    _bladerf_init_device(dev);

    return ret;
}

/*------------------------------------------------------------------------------
 * Si5338 register read / write functions
 */

int si5338_i2c_read(struct bladerf *dev, uint8_t address, uint8_t *val)
{
    int ret;
    struct uart_cmd uc;
    address &= 0x7f;
    uc.addr = address;
    uc.data = 0xff;
    ret = _bladerf_ioctl(dev, BLADE_SI5338_READ, &uc, sizeof(uc));
    *val = uc.data;
    return ret;
}

int si5338_i2c_write(struct bladerf *dev, uint8_t address, uint8_t val)
{
    struct uart_cmd uc;
    uc.addr = address;
    uc.data = val;
    return _bladerf_ioctl(dev, BLADE_SI5338_WRITE, &uc, sizeof(uc));
}

/*------------------------------------------------------------------------------
 * LMS register read / write functions
 */

int lms_spi_read(struct bladerf *dev, uint8_t address, uint8_t *val)
{
    int ret;
    struct uart_cmd uc;
    address &= 0x7f;
    uc.addr = address;
    uc.data = 0xff;
    ret = _bladerf_ioctl(dev, BLADE_LMS_READ, &uc, sizeof(uc));
    *val = uc.data;
    return ret;
}

int lms_spi_write(struct bladerf *dev, uint8_t address, uint8_t val)
{
    struct uart_cmd uc;
    uc.addr = address;
    uc.data = val;
    return _bladerf_ioctl(dev, BLADE_LMS_WRITE, &uc, sizeof(uc));
}

/*------------------------------------------------------------------------------
 * GPIO register read / write functions
 */
int gpio_read(struct bladerf *dev, uint32_t *val)
{
    int ret;
    int i;
    uint32_t rval;
    struct uart_cmd uc;
    rval = 0;
    for (i = 0; i < 4; i++) {
        uc.addr = i;
        uc.data = 0xff;
        ret = _bladerf_ioctl(dev, BLADE_GPIO_READ, &uc, sizeof(uc));
        if (ret) {
            if (errno == ETIMEDOUT) {
                ret = BLADERF_ERR_TIMEOUT;
            } else {
                ret = BLADERF_ERR_UNEXPECTED;
            }
            break;
        } else {
            rval |= uc.data << (i * 8);
        }
    }
    *val = rval;
    return ret;
}

int gpio_write(struct bladerf *dev, uint32_t val)
{
    struct uart_cmd uc;
    int ret;
    int i;

    i = 0;

    /* If we're connected at HS, we need to use smaller DMA transfers */
    if (dev->speed == 0)
        val |= BLADERF_GPIO_FEATURE_SMALL_DMA_XFER;

    for (i = 0; i < 4; i++) {
        uc.addr = i;
        uc.data = val >> (i * 8);
        ret = _bladerf_ioctl(dev, BLADE_GPIO_WRITE, &uc, sizeof(uc));
        if (ret) {
            if (errno == ETIMEDOUT) {
                ret = BLADERF_ERR_TIMEOUT;
            } else {
                ret = BLADERF_ERR_UNEXPECTED;
            }
            break;
        }
    }

    return ret;
}

/*------------------------------------------------------------------------------
 * VCTCXO DAC register write
 */
int dac_write(struct bladerf *dev, uint16_t val)
{
    struct uart_cmd uc;
    uc.word = val;
    int i;
    int ret;
    for (i = 0; i < 4; i++) {
        uc.addr = i;
        uc.data = val >> (i * 8);
        ret = _bladerf_ioctl(dev, BLADE_VCTCXO_WRITE, &uc, sizeof(uc));
        if (ret)
            break;
    }
    return ret;
}
