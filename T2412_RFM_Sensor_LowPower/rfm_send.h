#ifndef __RFM_SEND_H__
#define __RFM_SEND_H__

typedef struct
{
    char    radio_msg[MAX_MESSAGE_LEN];
    bool    flag_msg_was_sent;
} rfm_send_msg_st;



/// @brief  Get pointer to module data 
/// @note   
/// @return pointer to data
rfm_send_msg_st *rfm_send_get_data_ptr(void);

/// @brief  Send message
/// @param  message to send
/// @return
void rfm_send_radiate_msg( char *radio_msg );

void rfm_send_clr_flag_msg_was_sent(void);

bool rfm_send_get_flag_msg_was_sent(void);


#endif