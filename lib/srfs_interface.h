/* -*- c++ -*- */
/* 
 * Copyright 2013 Epiq Solutions.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef SRFS_INTERFACE_H
#define SRFS_INTERFACE_H

#include <stdio.h>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <stdint.h>
#include <netinet/in.h>

/** This file contains utilities to assist with communicating
    with the SRFS app */

namespace srfs {

    /** parameter data types */
    typedef enum 
    {
	SRFS_UINT64,
	SRFS_UINT32,
	SRFS_UINT16,
	SRFS_UINT8,
	SRFS_FLOAT,
	SRFS_ENUM,
    } SRFS_DATATYPES;
    
    /** container of a SRFS parameter */
    typedef struct 
    {
	SRFS_DATATYPES data_type;     /* type of data */
	void *p_value;                /* pointer to actual value of parameter */
	int64_t min_value;            /* min value of parameter */
	int64_t max_value;            /* max value of parameter */
	float resolution;             /* min resolution of parameter */
	const std::string *p_strings; /* array of strings (used for enums only) */
    } srfs_param_t;
    
    /* BINARY message format from SRFS */
    typedef struct __attribute__((__packed__))
    {
	uint16_t indicator; /* always 0 */
	uint16_t type;
	uint32_t length; /* length of message in octets */
	uint8_t message[0];
    } BINARY;
    
    /* BINARY IQ message from SRFS */
    typedef struct __attribute__((__packed__))
    {
	BINARY binary;
	uint32_t config_id;
	uint16_t format;
	uint64_t timestamp;
	int16_t iq[0];
    } BINARY_IQ;
    
    /* converts a BINARY message to the host format */
    bool BINARY_to_host( BINARY * const binary );
    /* converts a BINARY_IQ message to host format */
    bool BINARY_IQ_to_host( BINARY_IQ * const binary_iq );
    /* returns the length contained in the BINARY_IQ message */
    uint32_t BINARY_IQ_length_iq_in_pairs( const BINARY_IQ * const binary_iq );
    
    /* validates the value provided to ensure that it it falls within
       the range and resolution.  If it's valid, then it sets the p_value */
    bool set_param( srfs_param_t *p_param, void* p_value );
    
    /* reads in the string representation of the value and stores it
       in p_param */
    void update_param( srfs_param_t *p_param, const char* p_value );

    /* looks through *pStrings array for a match to pString.  invalid_index
       is the max number of elements contained on pStrings */
    uint32_t convert_str_to_enum( const char* pString, 
				  const std::string *pStrings, 
				  uint32_t invalid_index );
} // namespace SRFS

#endif
