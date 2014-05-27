/* -*- c++ -*- */

#define SIDEKIQ_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "sidekiq_swig_doc.i"

%{
#include "sidekiq/sidekiq_source_s.h"
%}

%include "sidekiq/sidekiq_defs.h"

%include "sidekiq/sidekiq_source_s.h"
GR_SWIG_BLOCK_MAGIC2(sidekiq, sidekiq_source_s);
