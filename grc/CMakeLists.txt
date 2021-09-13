#install(FILES
#    sidekiq_sidekiq_rx.yml
#    sidekiq_sidekiq_tx.yml
#    sidekiq_tx_burst_test.yml DESTINATION share/gnuradio/grc/blocks
# )

install(FILES
    sidekiq_sidekiq_rx.block.yml
    sidekiq_sidekiq_tx.block.yml DESTINATION share/gnuradio/grc/blocks
)
