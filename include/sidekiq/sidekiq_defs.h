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

#ifndef SIDEKIQ_DEFS_H
#define SIDEKIQ_DEFS_H

namespace gr {
    namespace sidekiq {

	/*!
	 * Status enable / disable 
	 */
	typedef enum 
	{ 
	    STATUS_ENABLED = 0,    
	    STATUS_DISABLED,       
	    STATUS_INVALID         
	} STATUS;

	/* Gain mode */
	typedef enum
	{
	    GAIN_MODE_MANUAL = 0,
	    GAIN_MODE_INVALID
	} GAIN_MODE;

    } // namespace sidekiq
} // namespace gr

#endif
