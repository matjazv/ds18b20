/* ds18b20.h
 *
 * Copyright (C) 2020 Matjaz Verbole
 *
 * This software may be modified and distributed under the terms
 * of the MIT license.  See the LICENSE file for details.
 */

#ifndef DS18B20_H_INCLUDED
#define DS18B20_H_INCLUDED

void ds18b20_init(uint8_t GPIO);
bool ds18b20_read_ROM(uint8_t *data);
bool ds18b20_match_ROM(uint8_t *ROMData);
bool ds18b20_skip_ROM(void);

#endif /* DS18B20_H_INCLUDED */
