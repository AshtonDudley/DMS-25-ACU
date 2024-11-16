void can_heartbeat(){
    // Transmit Heartbeat   
    TxHeartbeatData[0] = 0xFF;
    TxHeartbeatData[1] = 0xFF;
    TxHeartbeatData[2] = 0xFF;
    TxHeartbeatData[3] = 0xFF;
    TxHeartbeatData[4] = 0xFF;
    TxHeartbeatData[5] = 0xFF;
    TxHeartbeatData[6] = 0xFF;
    TxHeartbeatData[7] = 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeartbeat, TxHeartbeatData) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
}
