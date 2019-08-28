# M480BSP_SPI_Master_Slave
M480BSP_SPI_Master_Slave

update @ 2019/08/28

1. use SPI DMA interrupt to performace master and slave transmit

2. define ENABLE_SPI_MASTER , ENABLE_SPI_SLAVE , use M487 EVM x 2 for SPI master and slave

3. under SPI_TX in SPI_Master_PDMA_Enable and SPI_Slave_PDMA_Enable function , to change TX data

- Master TX buffer : g_au8MasterToSlaveTestPattern

- Slave TX buffer : g_au8SlaveToMasterTestPattern

4. check below array for RX data

- Master RX buffer : g_au8MasterRxBuffer

- Slave RX buffer : g_au8SlaveRxBuffer
