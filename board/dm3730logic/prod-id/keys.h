
#undef ID_KEY_START
#undef ID_KEY_ENTRY
#undef ID_KEY_END 

#if defined(ID_KEY_STRINGS)
/* This is the usage to build the keys for the compiler; we define
 * an array of strings whose index is the value */
#define ID_KEY_START static char *id_keys[] = {
#define ID_KEY_ENTRY(XX) #XX ,
#define ID_KEY_END };
#elif defined(ID_KEY_ENUMS)
/* This is the usage by people using the library to access the data */
#define ID_KEY_START typedef enum {
#define ID_KEY_ENTRY(XX) ID_KEY_ ## XX,
#define ID_KEY_END } id_keys_t;
#else
#error "Need either ID_KEY_INTERFACE or ID_KEY_COMPILER defined!"
#endif


/* There are some implied conventions here: */
/* - names of keys that contain other keys (dictionaries) end in "_group" */
/* - names of keys that provide a register setting end in "_reg"          */
/* - any keys that specify a unit of measure, include units in the name (ie. _mhz, _degf, _bytes) */

ID_KEY_START

/* Manufacturing unique data for each SOM */
ID_KEY_ENTRY(serialization_group)
ID_KEY_ENTRY(serial_number)
ID_KEY_ENTRY(lan_ethaddr1)
ID_KEY_ENTRY(lan_ethaddr2)
ID_KEY_ENTRY(lan_ethaddr3)
ID_KEY_ENTRY(lan_ethaddr4)
ID_KEY_ENTRY(nvs)

/* BOM Model number infromation */
ID_KEY_ENTRY(model_group)
ID_KEY_ENTRY(model_name)
ID_KEY_ENTRY(part_number)
ID_KEY_ENTRY(version_code)
ID_KEY_ENTRY(hardware_platform)

/* CPU specific information */
ID_KEY_ENTRY(cpu0_group)
ID_KEY_ENTRY(type)
ID_KEY_ENTRY(number)
ID_KEY_ENTRY(speed_mhz)
ID_KEY_ENTRY(temp_class)

/* CPU bus information */
ID_KEY_ENTRY(cpu0_bus_group)

/* DRAM bus information */
ID_KEY_ENTRY(dram_bus_group)
ID_KEY_ENTRY(sysconfig_reg)
ID_KEY_ENTRY(sharing_reg)
ID_KEY_ENTRY(dlla_ctrl_reg)
ID_KEY_ENTRY(cs_cfg_reg)
// ID_KEY_ENTRY(cs0_group) Used in the dram_bus_group, but key defined below after local_bus_group
// ID_KEY_ENTRY(cs1_group) Used in the dram_bus_group, but key defined below after local_bus_group
ID_KEY_ENTRY(mcfg_reg)
ID_KEY_ENTRY(mr_reg)
ID_KEY_ENTRY(rfr_ctrl_reg)
ID_KEY_ENTRY(emr2_reg)
ID_KEY_ENTRY(actim_ctrla_reg)
ID_KEY_ENTRY(actim_ctrlb_reg)
ID_KEY_ENTRY(power_reg)

/* GPMC keys */
ID_KEY_ENTRY(local_bus_group)
ID_KEY_ENTRY(cs0_group)
ID_KEY_ENTRY(cs1_group)
ID_KEY_ENTRY(cs2_group)
ID_KEY_ENTRY(cs3_group)
ID_KEY_ENTRY(cs4_group)
ID_KEY_ENTRY(cs5_group)
ID_KEY_ENTRY(cs6_group)
ID_KEY_ENTRY(config1_reg)
ID_KEY_ENTRY(config2_reg)
ID_KEY_ENTRY(config3_reg)
ID_KEY_ENTRY(config4_reg)
ID_KEY_ENTRY(config5_reg)
ID_KEY_ENTRY(config6_reg)
ID_KEY_ENTRY(config7_reg)

/* End of keys */
ID_KEY_END
