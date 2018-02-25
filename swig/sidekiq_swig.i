/* -*- c++ -*- */

#define SIDEKIQ_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "sidekiq_swig_doc.i"

%{
#include "sidekiq/sidekiq_rx.h"
#include "sidekiq/sidekiq_tx.h"
#include "sidekiq/tx_burst_test.h"
%}


%include "sidekiq/sidekiq_rx.h"
GR_SWIG_BLOCK_MAGIC2(sidekiq, sidekiq_rx);
%include "sidekiq/sidekiq_tx.h"
GR_SWIG_BLOCK_MAGIC2(sidekiq, sidekiq_tx);

%include "sidekiq/tx_burst_test.h"
GR_SWIG_BLOCK_MAGIC2(sidekiq, tx_burst_test);
