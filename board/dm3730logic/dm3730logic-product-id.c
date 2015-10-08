/*
 * (C) Copyright 2011
 * Logic Product Development <www.logicpd.com>
 * Peter Barada <peter.barada@logicpd.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/* Code to extract x-loader perntinent data from product ID chip */

#include <common.h>
#include <asm/arch/sys_proto.h>
#include "dm3730logic-ddr.h"
#include "dm3730logic-product-id.h"

#include "prod-id/interface.h"
#include "prod-id/id-errno.h"
#include "dm3730logic-id-data.h"

/* Fetch a byte of data from the ID data; in this case we know ID data is
 * builtin to the program (in dm3730logic-id-data.h) */
unsigned char id_fetch_byte(int offset, int *oor)
{
	/* If data is off end of know size then complain */
	if (offset >= sizeof(id_data_buf)) {
		id_printf("Attempt to read past end of buffer (offset %u >= size %u)\n", offset, sizeof(id_data_buf));
		*oor = -ID_ERANGE;
		return 0;  /* Force upper laer to recover */
	}

	*oor = ID_EOK;
	return id_data_buf[offset];
}

int id_printf(const char *fmt, ...)
{
	va_list args;
	char printbuffer[CFG_PBSIZE];

	va_start (args, fmt);
	/* For this to work, printbuffer must be larger than
	 * anything we ever want to print.
	 */
	vsprintf (printbuffer, fmt, args);
	va_end (args);
	/* Print the string */
	serial_puts (printbuffer);

	return 0;
}

void id_error(const char *fmt, ...)
{
	va_list args;
	char printbuffer[CFG_PBSIZE];

	va_start (args, fmt);
	/* For this to work, printbuffer must be larger than
	 * anything we ever want to print.
	 */
	vsprintf (printbuffer, fmt, args);
	va_end (args);
	/* Print the string */
	serial_puts (printbuffer);

}

struct id_data id_data;
static int found_id_data;
/* Initialize the product ID data and return 0 if found */
static int product_id_init(void)
{
	int ret;

	ret = id_startup(&id_data);
	if (ret != ID_EOK) {
		return -1;
	}

	return 0;
}

int dm3730logic_has_product_id(void)
{
	if (!found_id_data) {
		if (!product_id_init()) {
			found_id_data = 1;
		}
	} else
		printf("%s:%d\n", __FUNCTION__, __LINE__);
	return found_id_data;
}

id_keys_t dram_bus_group_keys[] = {
	ID_KEY_sysconfig_reg,
	ID_KEY_sharing_reg,
	ID_KEY_power_reg,
	ID_KEY_cs_cfg_reg,
};

int dram_bus_group_values[ARRAY_SIZE(dram_bus_group_keys)];

id_keys_t dram_cs_group_keys[] = {
	ID_KEY_mcfg_reg,
	ID_KEY_mr_reg,
	ID_KEY_rfr_ctrl_reg,
	ID_KEY_emr2_reg,
	ID_KEY_actim_ctrla_reg,
	ID_KEY_actim_ctrlb_reg,
	ID_KEY_dlla_ctrl_reg,
};

int dram_cs_group_values[ARRAY_SIZE(dram_cs_group_keys)];

int dm3730logic_extract_ddr_timing(struct sdram_timings *timing)
{
	int ret;
	struct id_cookie cookie, dram_bus_group_cookie;

	if (!found_id_data)
		return -1;

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		return ret;
	}

	/* find /cpu0_bus_group from root */
	ret = id_find_dict(&cookie, ID_KEY_cpu0_bus_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	/* find /dram_bus_group from /cpu0_bus_group */
	ret = id_find_dict(&cookie, ID_KEY_dram_bus_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	dram_bus_group_cookie = cookie;
	ret = id_find_numbers(&dram_bus_group_cookie, dram_bus_group_keys, ARRAY_SIZE(dram_bus_group_keys), dram_bus_group_values);
	if (ret != ID_EOK) {
		return ret;
	}

	timing->name=NULL; /* No name in SDRAM data */
	timing->sysconfig = dram_bus_group_values[0];
	timing->sharing = dram_bus_group_values[1];
	timing->power = dram_bus_group_values[2];
	timing->cfg=dram_bus_group_values[3];


	ret = id_find_dict(&dram_bus_group_cookie, ID_KEY_cs0_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	ret = id_find_numbers(&dram_bus_group_cookie, dram_cs_group_keys, ARRAY_SIZE(dram_cs_group_keys), dram_cs_group_values);
	if (ret != ID_EOK) {
		return ret;
	}
	timing->mcfg[0] = dram_cs_group_values[0];
	timing->mr[0] = dram_cs_group_values[1];
	timing->rfr[0] = dram_cs_group_values[2];
	timing->emr[0] = dram_cs_group_values[3];
	timing->actima[0] = dram_cs_group_values[4];
	timing->actimb[0] = dram_cs_group_values[5];
	timing->dlla = dram_cs_group_values[6];
	timing->offset = ((timing->mcfg[0]) >> 8 & 0x3ff) << 21;

	ret = id_find_dict(&dram_bus_group_cookie, ID_KEY_cs1_group, IDENUM_DICT);
	if (ret == ID_EOK) {
		ret = id_find_numbers(&dram_bus_group_cookie, dram_cs_group_keys, ARRAY_SIZE(dram_cs_group_keys), dram_cs_group_values);
		if (ret != ID_EOK) {
			return ret;
		}
		timing->mcfg[1] = dram_cs_group_values[0];
		timing->mr[1] = dram_cs_group_values[1];
		timing->rfr[1] = dram_cs_group_values[2];
		timing->emr[1] = dram_cs_group_values[3];
		timing->actima[1] = dram_cs_group_values[4];
		timing->actimb[1] = dram_cs_group_values[5];
		timing->dllb = timing->dlla;
	} else {
		timing->mcfg[1] = 0;
		timing->mr[1] = 0;
		timing->rfr[1] = 0;
		timing->emr[1] = 0;
		timing->actima[1] = 0;
		timing->actimb[1] = 0;
		timing->dllb = 0;
	}


	/* Extract /sysconfig_reg */

	
	return 0;
}

int serialization_info(void)
{
	int ret;
	struct id_cookie cookie;
	u32 part_number;
	char model_name[32];
	u32 model_name_size;
	char serial_number[10];
	u32 serial_number_size;

	if (!found_id_data) {
		return -1;
	}

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		printf("%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* find /serialization_group from root */
	ret = id_find_dict(&cookie, ID_KEY_serialization_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		printf("%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* Find serial_number */
	serial_number_size = sizeof(serial_number);
	ret = id_find_string(&cookie, ID_KEY_serial_number, serial_number, &serial_number_size);
	if (ret != ID_EOK) {
		printf("%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* Reinitialise cookie back to the root */
	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		printf("%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* find /model_group from root */
	ret = id_find_dict(&cookie, ID_KEY_model_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		printf("%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* Find part number */
	ret = id_find_number(&cookie, ID_KEY_part_number, &part_number);
	if (ret != ID_EOK) {
		printf("%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	/* Find model name */
	model_name_size = sizeof(model_name);
	ret = id_find_string(&cookie, ID_KEY_model_name, model_name, &model_name_size);
	if (ret != ID_EOK) {
		printf("%s:%d ret %d\n", __FUNCTION__, __LINE__, ret);
		return ret;
	}

	printf("Part Number  : %u\n", part_number);
	printf("Model Name   : %.*s\n", model_name_size, model_name);
	printf("Serial Number: %.*s\n", serial_number_size, serial_number);
	return 0;
}

/* Extract GPMC timings for particular CS register */
id_keys_t gpmc_ncs_keys[] = {
	ID_KEY_cs0_group,
	ID_KEY_cs1_group,
	ID_KEY_cs2_group,
	ID_KEY_cs3_group,
	ID_KEY_cs4_group,
	ID_KEY_cs5_group,
	ID_KEY_cs6_group,
};

id_keys_t gpmc_config_reg_keys[] = {
	ID_KEY_config1_reg,
	ID_KEY_config2_reg,
	ID_KEY_config3_reg,
	ID_KEY_config4_reg,
	ID_KEY_config5_reg,
	ID_KEY_config6_reg,
	ID_KEY_config7_reg,
};

int dm3730logic_extract_gpmc_timing(int cs, int *config_regs)
{
	int ret;
	struct id_cookie cookie;
	// int gpmc_config_values[ARRAY_SIZE(gpmc_config_reg_keys)];

	if (!found_id_data)
		return -1;

	ret = id_init_cookie(&id_data, &cookie);
	if (ret != ID_EOK) {
		return ret;
	}

	/* find /cpu0_bus_group from root */
	ret = id_find_dict(&cookie, ID_KEY_cpu0_bus_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	/* find /local_bus_group from /cpu0_bus_group */
	ret = id_find_dict(&cookie, ID_KEY_local_bus_group, IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	/* Now look for the particular chip select group */
	ret = id_find_dict(&cookie, gpmc_ncs_keys[cs], IDENUM_DICT);
	if (ret != ID_EOK) {
		return ret;
	}

	/* We have the group, now extract all the config registers */
	ret = id_find_numbers(&cookie, gpmc_config_reg_keys, ARRAY_SIZE(gpmc_config_reg_keys), config_regs);

	return ret;
}
