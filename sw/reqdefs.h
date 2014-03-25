/* Name:    reqdefs.h
 * Project: AVR002:	Multi-switch event recorder with USB support
 * Author:  Vladimir Stankovic
 * Creation Date: 2014-03-22
 * Copyright: N/A
 * License: N/A
 */

#ifndef REQDEFS_H
#define REQDEFS_H

#define REQ_GET_HEADER          0x00

#define REQ_GET_DATA1           0x10
#define REQ_GET_DATA2           0x11
#define REQ_GET_DATA3           0x12
#define REQ_GET_DATA4           0x13

#define REQ_SET_DATA1           0x20
#define REQ_SET_DATA2           0x21
#define REQ_SET_DATA3           0x22
#define REQ_SET_DATA4           0x23

#endif // REQDEFS_H
