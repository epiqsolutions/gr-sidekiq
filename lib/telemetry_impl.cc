/* -*- c++ -*- */
/*
 * Copyright 2023 gr-sidekiq author.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <inttypes.h>
#include "telemetry_impl.h"
#include <gnuradio/io_signature.h>
#include "sidekiq_api.h"

namespace gr {
namespace sidekiq {

telemetry::sptr telemetry::make(
        int input_card,
        int temp_enabled,
        int imu_enabled) 
{
  return gnuradio::make_block_sptr<telemetry_impl>(
          input_card,
          temp_enabled,
          imu_enabled);
}

/*
 * The private constructor
 */
telemetry_impl::telemetry_impl(
        int input_card,
        int temp_enabled,
        int imu_enabled) 
    : block("telemetry",
                     gr::io_signature::make(0 /* min inputs */,
                                            0 /* max inputs */,
                                            0),
                     gr::io_signature::make(0, 0, 0))
{
    int status;

    this->card = input_card;
    this->temp_enabled = temp_enabled;
    this->imu_enabled = imu_enabled;


    printf("card %d, temp_enabled %d, imu_enabled %d\n", input_card, temp_enabled, imu_enabled); 
    /* initialize libsidekiq */
    status = skiq_init(skiq_xport_type_pcie, skiq_xport_init_level_full, &card, 1);
    if (status != 0)
    {
        if (status != -EEXIST)
        {
            d_logger->error( "Error: unable to initialize libsidekiq with status {}", status);
            throw std::runtime_error("Failure: skiq_init");
        }
        else
        {
            d_logger->info("Info: If not running Transceive Mode, then this is an error");
        }
    }
    else
    {
        d_logger->info("Info: libsidkiq initialized successfully");
    }

    message_port_register_in(pmt::mp("temp"));
    set_msg_handler(pmt::mp("temp"),
                    [this](const pmt::pmt_t& msg) { this->temp(msg); });
      
    message_port_register_in(pmt::mp("imu"));
    set_msg_handler(pmt::mp("imu"),
            [this](const pmt::pmt_t& msg) { this->imu(msg); });


    
}

void telemetry_impl::temp(const pmt::pmt_t& msg)
{
    int status = 0;
    int8_t temp = 0;

    if (this->temp_enabled != 0)
    {
        /* read the temperature */
        status = skiq_read_temp( card, &temp );
        if ( status == 0 )
        {
            printf("Info: on-board temperature is %" PRIi8 " degrees Celsius\n", temp);
        }
        else if ( status == -EAGAIN )
        {
            fprintf(stderr, "Error: on-board temperature is temporarily unavailable\n");
        }
        else if ( status == -ENODEV )
        {
            fprintf(stderr, "Warning: on-board temperature may not available at configured "
                    "skiq_xport_init_level, try specifying --full in the command argument list\n");
        }
        else
        {
            fprintf(stderr, "Error: failed to read on-board temperature (result code %" PRIi32
                    ")\n", status);
        }
    }

}

/**************************************************************************************************/
/* This functions reads two 8-bit registers and returns the 16-bit two's complement result

   @param card_idx  which sidekiq card to read, already initialized
   @param reg   base register address (reads reg, reg+1)
   @param int *val    returns result
   @return int - indicated status of read

*/
int read_accel_reg_word( uint8_t card_idx, uint8_t reg, int16_t *val)
{
    int32_t status = 0;
    uint8_t low_byte = 0, high_byte = 0;
    int16_t result = 0;

    status = skiq_read_accel_reg( card_idx, reg, &high_byte, 1 );
    if ( status == 0 )
    {
        status = skiq_read_accel_reg( card_idx, reg+1, &low_byte, 1);
    }

    if ( status == 0 )
    {
        result = (((int16_t)high_byte)<<8) | low_byte;
        {
            result = result - 0x10000;
        }
        *val = result;
    }
    else
    {
        fprintf(stderr, "Error: unable to read register 0x%02x (%" PRIi32 ")\n",
                reg, status);
    }

    return status;
}

void telemetry_impl::imu(const pmt::pmt_t& msg)
{
    int status = 0;
    int16_t acc_x = 0, acc_y = 0, acc_z = 0, gyro_x = 0, gyro_y = 0, gyro_z = 0 ;
    uint8_t val = 0;


    if (this->imu_enabled != 0)
    {
        /* set reg 0x6b to 0x01 to take card out of sleep/standby */
        val = 0x01;
        status = skiq_write_accel_reg( card, 0x6b, &val, 1);
        if ( status != 0 )
        {
            fprintf(stderr, "Error: unable to take IMU out of sleep / standby (result code %"
                    PRIi32 ")\n", status);
            return;
        }

        if ( status == 0 ) status = read_accel_reg_word( card, 0x3b, &acc_x);
        if ( status == 0 ) status = read_accel_reg_word( card, 0x3d, &acc_y);
        if ( status == 0 ) status = read_accel_reg_word( card, 0x3f, &acc_z);
        if ( status == 0 ) status = read_accel_reg_word( card, 0x43, &gyro_x);
        if ( status == 0 ) status = read_accel_reg_word( card, 0x45, &gyro_y);
        if ( status == 0 ) status = read_accel_reg_word( card, 0x47, &gyro_z);

        if ( status == 0)
        {
            printf("AX: %6d  AY: %6d  AZ: %6d  GX: %6d  GY: %6d  GZ:  %6d\n",
                    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
        }
        else
        {
            fprintf(stderr, "Error: unable to read IMU measurement (result code %"
                    PRIi32 ")\n", status);
            return;
        }
    }


}


/*
 * Our virtual destructor.
 */
telemetry_impl::~telemetry_impl() {}


} /* namespace sidekiq */
} /* namespace gr */
