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

#include "srfs_interface.h"

#include <math.h>
#include <string.h>
#include <stdexcept>

namespace srfs {

    template <class T> bool check_range( T *p_param, T min, T max, T val );
    template <class T> bool check_resolution( T *p_param, T res, T val );

    bool
    BINARY_to_host( BINARY * const binary )
    {
	bool return_value = false;
	if ( NULL != binary ) {
	    binary->indicator = ntohs( binary->indicator );
	    binary->type = ntohs( binary->type );
	    binary->length = ntohl( binary->length );
	    return_value = true;
	}
	return ( return_value );
    } /* end BINARY_to_host */
    
    uint32_t
    BINARY_IQ_length_iq_in_pairs( const BINARY_IQ * const binary_iq )
    {
	uint32_t length_iq = 0;
	if ( NULL != binary_iq ) {
	    length_iq = ( binary_iq->binary.length -
			  ( sizeof(*binary_iq) -
			    sizeof(binary_iq->binary) ) ) /
		( 2 * sizeof(*binary_iq->iq) ); /* FIXME depends on type */
	}
	return ( length_iq );
    } /* end BINARY_IQ_length_iq */
    
    bool
    BINARY_IQ_to_host ( BINARY_IQ * const binary_iq )
    {
	bool return_value = false;
	uint32_t i;
	uint32_t length;
	
	if ( NULL != binary_iq ) {
	    return_value = BINARY_to_host( &binary_iq->binary );
	    binary_iq->config_id = be32toh( binary_iq->config_id );
	    binary_iq->format = be16toh( binary_iq->format );
	    binary_iq->timestamp = be64toh( binary_iq->timestamp );
	    length = BINARY_IQ_length_iq_in_pairs(binary_iq);
	    
	    /* NOTE: it is the responsibility of the caller to convert the actual data */
	}
	return ( return_value );
    } /* end BINARY_IQ_to_host */
    
    void
    update_param( srfs_param_t *p_param, const char *p_value )
    {
	if( p_value != NULL ) {
	    switch( p_param->data_type ) {
		case SRFS_UINT64:
		    sscanf( p_value, "%lu", 
			    (uint64_t*)(p_param->p_value) );
		    break;
		    
		case SRFS_UINT32:
		    sscanf( p_value, "%u", 
			    (uint32_t*)(p_param->p_value) );
		    break;
		    
		case SRFS_UINT16:
		    sscanf( p_value, "%hu", 
			    (uint16_t*)(p_param->p_value) );
		    break;
		    
		case SRFS_UINT8:
		    sscanf( p_value, "%hhu", 
			    (uint8_t*)(p_param->p_value) );
		    break;
		    
		case SRFS_FLOAT:
		    sscanf( p_value, "%f", 
			    (float*)(p_param->p_value) );
		    break;
		    
		case SRFS_ENUM:
		    *((uint32_t*)(p_param->p_value)) = 
			convert_str_to_enum( p_value,
					     p_param->p_strings, 
					     p_param->max_value );
		    break;
		
		default:
		    throw std::invalid_argument("invalid data_type on update");
		    break;
	    }
	}
    }

    template <class T>
    bool 
    set_param( T *p_param, T min, T max, T res, T val )
    {
	bool b_valid = false;

	b_valid = check_range( p_param, min, max, val );
	b_valid = check_resolution( p_param, res, val );

	if( b_valid )
	{
	    *p_param = val;
	}

	return b_valid;
    }

    template <class T>
    bool 
    check_range( T *p_param, T min, T max, T val )
    {
	bool b_valid = false;

	if( (val >= min) && (val <= max) )
	{
	    b_valid = true;
	}
	else
	{
	    throw std::out_of_range("value exceeds min or max");
	}
	return b_valid;
    }

    template <class T>
    bool 
    check_resolution( T *p_param, T res, T val )
    {
	bool b_valid = false;

	if( (val % res) == 0 )
	{
	    b_valid = true;
	}
	else
	{
	    throw std::out_of_range("value exceeds resolution");
	}
	return b_valid;
    }
    
    bool
    set_param( srfs_param_t *p_param, void *p_value )
    {
	bool b_valid = false;

	switch( p_param->data_type ) {
	    case SRFS_UINT64:
		b_valid = 
		    set_param<uint64_t>( (uint64_t*)(p_param->p_value),
					 (uint64_t)(p_param->min_value),
					 (uint64_t)(p_param->max_value),
					 (uint64_t)(p_param->resolution),
					 *((uint64_t*)(p_value)) );
		break;

	    case SRFS_UINT32:
		b_valid = 
		    set_param<uint32_t>( (uint32_t*)(p_param->p_value),
					 (uint32_t)(p_param->min_value),
					 (uint32_t)(p_param->max_value),
					 (uint32_t)(p_param->resolution),
					 *((uint32_t*)(p_value)) );
		break;
		
	    case SRFS_UINT16:
		b_valid = 
		    set_param<uint16_t>( (uint16_t*)(p_param->p_value),
					  (uint16_t)(p_param->min_value),
					  (uint16_t)(p_param->max_value),
					  (uint16_t)(p_param->resolution),
					 *((uint16_t*)(p_value)) );
		break;
		
	    case SRFS_UINT8:
		b_valid = 
		    set_param<uint8_t>( (uint8_t*)(p_param->p_value),
					 (uint8_t)(p_param->min_value),
					 (uint8_t)(p_param->max_value),
					 (uint8_t)(p_param->resolution),
					 *((uint8_t*)(p_value)) );
		break;
		
	    case SRFS_FLOAT:
		b_valid = 
		    check_range( (float*)(p_param->p_value),
				 (float)(p_param->min_value),
				 (float)(p_param->max_value),
				 *((float*)(p_value)) );
		/* check the resolution manually */
		if( b_valid == true )
		{
		    if( fmod( *(float*)(p_param->p_value),
			      (float)(p_param->resolution) ) == 0 )
		    {
			*((float*)(p_param->p_value)) = *((float*)(p_value));
		    }
		    else
		    {
			char e[200];
			b_valid = false;
			throw std::out_of_range("value exceeds resolution");
		    }
		}
		break;
		
	    case SRFS_ENUM:
		// just treat as uint32 since we defined our ranges appropriately already
		b_valid = 
		    set_param<uint32_t>( (uint32_t*)(p_param->p_value),
					 (uint32_t)(p_param->min_value),
					 (uint32_t)(p_param->max_value),
					 (uint32_t)(p_param->resolution),
					 *((uint32_t*)(p_value)) );
		break;
		
	    default:
		throw std::invalid_argument("invalid data_type on set");
		break;
	}
	return b_valid;
    }
    
    uint32_t
    convert_str_to_enum( const char* pString, 
			 const std::string *pStrings, 
			 uint32_t invalid_index )
    {
	uint32_t index = invalid_index;
	
	for( index=0; index<invalid_index; index++ ) {
	    if( strncmp(pString, 
			pStrings[index].c_str(),
			strlen(pStrings[index].c_str())) == 0 ) {
		break;
	    }
	}
	if( invalid_index == index ) {
	    throw std::out_of_range("out_of_range, string not in enum");
	}
	return index;
    }
} // namespace srfs


