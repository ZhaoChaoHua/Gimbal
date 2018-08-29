#include "parameter.h"
#include "stm32f40x_define.h"
extern "C"
{
#include "e2pfs.h"
}

#define Debug(fmt, args ...)  do {rt_kprintf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)

// some useful progmem macros
#define PGM_UINT8(addr) pgm_read_byte((const prog_char *)addr)
#define PGM_UINT16(addr) pgm_read_word((const uint16_t *)addr)
#define PGM_FLOAT(addr) pgm_read_float((const float *)addr)
#define PGM_POINTER(addr) pgm_read_pointer((const void *)addr)

#define GROUP_ID(grpinfo, base, i, shift) ((base)+(((uint16_t)PGM_UINT8(&grpinfo[i].idx))<<(shift)))
// number of rows in the _var_info[] table
uint8_t AP_Param::_num_vars;

// storage and naming information about all types that can be saved
const AP_Param::Info *AP_Param::_var_info;

// return the storage size for a AP_PARAM_* type
uint8_t AP_Param::type_size(enum ap_var_type type)
{
    switch (type) {
    case AP_PARAM_NONE:
    case AP_PARAM_GROUP:
        return 0;
    case AP_PARAM_INT8:
        return 1;
    case AP_PARAM_INT16:
        return 2;
    case AP_PARAM_INT32:
        return 4;
    case AP_PARAM_FLOAT:
        return 4;
    }
    Debug("unknown type %u\n", type);
    return 0;
}

void AP_Param::erase_all(void)
{
    e2pfs_format();
}

// validate a group info table
bool AP_Param::check_group_info(const struct AP_Param::GroupInfo *  group_info, uint8_t prefix_length)
{
    uint8_t type;
    int8_t max_idx = -1;
    for (uint8_t i=0;
         (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
         i++)
    {
        uint8_t idx = PGM_UINT8(&group_info[i].idx);
        if ((int8_t)idx <= max_idx) {
            Debug("indexes must be in increasing order in %S", group_info[i].name);
            return false;
        }
        max_idx = (int8_t)idx;
        uint8_t size = type_size((enum ap_var_type)type);
        if (size == 0) {
            Debug("invalid type in %S", group_info[i].name);
            return false;
        }
        if (prefix_length + strlen_P(group_info[i].name) > 16) {
            Debug("suffix is too long in %S", group_info[i].name);
            return false;
        }
    }
    return true;
}

// validate the _var_info[] table
bool AP_Param::check_var_info(void)
{
    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = PGM_UINT8(&_var_info[i].type);
        uint8_t key = PGM_UINT8(&_var_info[i].key);
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[i].group_info);
            if (group_info == NULL ||
                !check_group_info(group_info, strlen_P(_var_info[i].name))) {
                return false;
            }
        } else {
            uint8_t size = type_size((enum ap_var_type)type);
            if (size == 0) {
                // not a valid type - the top level list can't contain
                // AP_PARAM_NONE
                return false;
            }
        }
    }
    // we no longer check if total_size is larger than _eeprom_size,
    // as we allow for more variables than could fit, relying on not
    // saving default values
    return true;
}

// find the info structure for a variable in a group
const struct AP_Param::Info *AP_Param::find_var_info_group(const struct GroupInfo * group_info,
                                                           uint8_t                  vindex,
                                                           uint32_t *               group_element,
                                                           const struct GroupInfo **group_ret,
                                                           uint8_t *                idx) const
{
    uintptr_t base = PGM_POINTER(&_var_info[vindex].ptr);
//    uint8_t type;
	// 遍历组内数据
    for (uint8_t i=0;
         PGM_UINT8(&group_info[i].type) != AP_PARAM_NONE;
         i++) {
        uintptr_t ofs = PGM_POINTER(&group_info[i].offset);
        if ((uintptr_t) this == base + ofs) {
            *group_element = GROUP_ID(group_info, 0, i, 0); //&group_info[i].idx
            *group_ret = &group_info[i];
            *idx = 0;
            return &_var_info[vindex];
        } 
    }
    return NULL;
}

// Find a variable by name in a group
AP_Param *AP_Param::find_group(const char *name, uint8_t vindex, const struct GroupInfo *group_info, enum ap_var_type *ptype)
{
    uint8_t type;
    for (uint8_t i=0;
         (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
         i++) {
        if (strcasecmp_P(name, group_info[i].name) == 0) {
            uintptr_t p = PGM_POINTER(&_var_info[vindex].ptr);
            *ptype = (enum ap_var_type)type;
            return (AP_Param *)(p + PGM_POINTER(&group_info[i].offset));
        }
    }
    return NULL;
}

// find the info structure for a variable
const struct AP_Param::Info *AP_Param::find_var_info(uint32_t *                 group_element,
                                                     const struct GroupInfo **  group_ret,
                                                     uint8_t *                  idx)
{
    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = PGM_UINT8(&_var_info[i].type);
        uintptr_t base = PGM_POINTER(&_var_info[i].ptr);
        if (type == AP_PARAM_GROUP) {
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[i].group_info);
            const struct AP_Param::Info *info;
            info = find_var_info_group(group_info, i, group_element, group_ret, idx);
            if (info != NULL) {
                return info;
            }
        } else if (base == (uintptr_t) this) {
            *group_element = 0;
            *group_ret = NULL;
            *idx = 0;
            return &_var_info[i];
        }
    }
    return NULL;
}

// Find a variable by name.
//
AP_Param *AP_Param::find(const char *name, enum ap_var_type *ptype)
{
    for (uint8_t i=0; i<_num_vars; i++) {
        uint8_t type = PGM_UINT8(&_var_info[i].type);
        if (type == AP_PARAM_GROUP) {
            uint8_t len = strlen_P(_var_info[i].name);
            if (strncmp_P(name, _var_info[i].name, len) != 0) {
                continue;
            }
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[i].group_info);
            AP_Param *ap = find_group(name + len, i, group_info, ptype);
            if (ap != NULL) {
                return ap;
            }
            // we continue looking as we want to allow top level
            // parameter to have the same prefix name as group
            // parameters, for example CAM_P_G
        } else if (strcasecmp_P(name, _var_info[i].name) == 0) {
            *ptype = (enum ap_var_type)type;
            return (AP_Param *)PGM_POINTER(&_var_info[i].ptr);
        }
    }
    return NULL;
}

const float *AP_Param::find_def_value_ptr(const char *name)
{
    enum ap_var_type ptype;
    AP_Param *vp = find(name, &ptype);
    if (vp == NULL) {
        return NULL;
    }
    uint32_t group_element;
    const struct GroupInfo *ginfo;
    uint8_t gidx;
    const struct AP_Param::Info *info = vp->find_var_info(&group_element, &ginfo, &gidx);
    if (info == NULL) {
        return NULL;
    }
    if (ginfo != NULL) {
        return &ginfo->def_value;
    }
    return &info->def_value;
}


// Find a object by name.
//
AP_Param *AP_Param::find_object(const char *name)
{
    for (uint8_t i=0; i<_num_vars; i++) {
        if (strcasecmp_P(name, _var_info[i].name) == 0) {
            return (AP_Param *)PGM_POINTER(&_var_info[i].ptr);
        }
    }
    return NULL;
}

// set a AP_Param variable to a specified value
void AP_Param::set_value(enum ap_var_type type, void *ptr, float *value)
{
    switch (type) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)ptr)->set(*value);
        break;
    case AP_PARAM_INT16:
        ((AP_Int16 *)ptr)->set(*value);
        break;
    case AP_PARAM_INT32:
        ((AP_Int32 *)ptr)->set(*value);
        break;
    case AP_PARAM_FLOAT:
        ((AP_Float *)ptr)->set(*value);
        break;
    default:
        break;
    }
}

void* AP_Param::get_value(enum ap_var_type type, void* ptr)
{
    void *p_value;
    switch (type) {
        case AP_PARAM_INT8:
            p_value = (void *)&(((AP_Int8 *)ptr)->get());
            break;
        case AP_PARAM_INT16:
            p_value = (void *)&(((AP_Int16 *)ptr)->get());
            break;
        case AP_PARAM_INT32:
            p_value = (void *)&(((AP_Int32 *)ptr)->get());
            break;
        case AP_PARAM_FLOAT:
            p_value = (float *)&(((AP_Float *)this)->get());
            break;
        default:
            break;
    }

	return p_value;
}

/// cast a variable to a float given its type
float AP_Param::cast_to_float(enum ap_var_type type) const
{
    switch (type) {
    case AP_PARAM_INT8:
        return ((AP_Int8 *)this)->cast_to_float();
    case AP_PARAM_INT16:
        return ((AP_Int16 *)this)->cast_to_float();
    case AP_PARAM_INT32:
        return ((AP_Int32 *)this)->cast_to_float();
    case AP_PARAM_FLOAT:
        return ((AP_Float *)this)->cast_to_float();
    default:
        return NAN;
    }
}

// Save the current variable to EEPROM, if supported
//
bool AP_Param::save(void)
{
    int ret;
    int fd;
    E2P_File *file_n;
    uint32_t wlen;

    uint32_t group_element = 0;
    const struct GroupInfo *ginfo;
    uint8_t idx;
    const struct AP_Param::Info *info = find_var_info(&group_element, &ginfo, &idx);
    const AP_Param *ap;

    if (info == NULL) {
        // we don't have any info on how to store it
        return false;
    }
    struct Param_header phdr;
    
    // create the header we will use to store the variable
    if (ginfo != NULL) {
        phdr.type = PGM_UINT8(&ginfo->type);
    } else {
        phdr.type = PGM_UINT8(&info->type);
    }
    phdr.key  = PGM_UINT8(&info->key);
    phdr.group_element = group_element;
    phdr.id = phdr.key + phdr.group_element;
    
    ap = this;
    if (idx != 0) {
        ap = (const AP_Param *)((uintptr_t)ap) - (idx*sizeof(float));
    }
	
    // 存储参数到EEPROM
    if (phdr.type <= AP_PARAM_FLOAT){
        float value = cast_to_float((enum ap_var_type)phdr.type);

		// 存入E2PROM
		ret = e2pfs_fexist(phdr.id);
		if(ret < 0){
			 rt_kprintf("error\r\n");
		}

		fd = e2pfs_open(phdr.id);
		if(fd < 0){
			 rt_kprintf("[err]File open ERROR\n");
			 return false;
		}
        switch (phdr.type) {
            case AP_PARAM_INT8:{
                int8_t v_int8 = value;
                wlen = e2pfs_write(fd, &v_int8, type_size((enum ap_var_type)phdr.type));
            }
                break;
            case AP_PARAM_INT16:{
                int16_t v_int16 = value;
                wlen = e2pfs_write(fd, &v_int16, type_size((enum ap_var_type)phdr.type));
            }
                break;
            case AP_PARAM_INT32:{
                int32_t v_int32 = value;
                wlen = e2pfs_write(fd, &v_int32, type_size((enum ap_var_type)phdr.type));
            }
                break;
            case AP_PARAM_FLOAT:{
                float v_float = value;
                wlen = e2pfs_write(fd, &v_float, type_size((enum ap_var_type)phdr.type));
            }
                break;
            }

		file_n = FD2FILEP(fd);
		file_n->f_info.i_size = wlen;
		file_n->f_info._default = 1;
		
		e2pfs_close(fd);
		
		return true;
    }
    return false;
}

// Load the variable from EEPROM, if supported
//
bool AP_Param::load(void)
{
//    uint32_t size;
    uint32_t rlen;
    int fd;
    int ret;
    E2P_File *file_n;
	
	uint8_t value_ptr[4];
    float set_value;
	
	struct Param_header phdr;
    
	//用于存储数据
    uint32_t group_element = 0;
    const struct GroupInfo *ginfo;
    uint8_t idx;
	
    const struct AP_Param::Info *info = find_var_info(&group_element, &ginfo, &idx);
    if (info == NULL) {
        // we don't have any info on how to load it
        return false;
    }

    // create the header we will use to match the variable
    if (ginfo != NULL) {
        phdr.type = PGM_UINT8(&ginfo->type);
    } else {
        phdr.type = PGM_UINT8(&info->type);
    }
    phdr.key  = PGM_UINT8(&info->key);
    phdr.group_element = group_element;
    phdr.id = phdr.key + phdr.group_element;	//参数id

    AP_Param *ap = this;

    if (idx != 0) { //idx = 0
        ap = (AP_Param *)((uintptr_t)ap) - (idx*sizeof(float));
    }
	
	if (phdr.type <= AP_PARAM_FLOAT){
		ret = e2pfs_fexist(phdr.id);
		if(ret < 0){
			rt_kprintf("[err]No such file\n");
		}
		fd = e2pfs_open(phdr.id);
		if(fd < 0){
            if (info->group_info == NULL){
				set_value = info->def_value;
			}
			else{
				set_value = ginfo->def_value;
			}
			rt_kprintf("[err]File open ERROR\n");
			goto set_value;
		}

		file_n = FD2FILEP(fd);
		
		if(file_n->f_info._default == 0){
			//rt_kprintf("[err]Parameter not set yet, re-set to default value\n");
			if (info->group_info == NULL){
				set_value = info->def_value;
			}
			else{
				set_value = ginfo->def_value;
			}
			e2pfs_close(fd);
		}
		else{
//			size = file_n->f_info.i_size;
			
			rlen = e2pfs_read(fd, value_ptr, type_size((enum ap_var_type)phdr.type));
			if(rlen == 0){
				rt_kprintf("[err]parameter read error\n");
				e2pfs_close(fd);
				return RT_ERROR;
			}
			e2pfs_close(fd);
            switch (phdr.type) {
            case AP_PARAM_INT8:
                int8_t v_int8;
                rt_memcpy(&v_int8,value_ptr,sizeof(int8_t));
                ((AP_Int8 *)ap)->set(v_int8);
                break;
            case AP_PARAM_INT16:
                int16_t v_int16;
                rt_memcpy(&v_int16,value_ptr,sizeof(int16_t));
                ((AP_Int16 *)ap)->set(v_int16);
                break;
            case AP_PARAM_INT32:
                int32_t v_int32;
                rt_memcpy(&v_int32,value_ptr,sizeof(int32_t));
                ((AP_Int32 *)ap)->set(v_int32);
                break;
            case AP_PARAM_FLOAT:
                float v_float;
                rt_memcpy(&v_float,value_ptr,sizeof(float));
                ((AP_Float *)ap)->set(v_float);
                break;
            }
		}
        return true;
	}
set_value:
    switch (phdr.type) {
    case AP_PARAM_INT8:
        ((AP_Int8 *)ap)->set(set_value);
        break;
    case AP_PARAM_INT16:
        ((AP_Int16 *)ap)->set(set_value);
        break;
    case AP_PARAM_INT32:
        ((AP_Int32 *)ap)->set(set_value);
        break;
    case AP_PARAM_FLOAT:
        ((AP_Float *)ap)->set(set_value);
        break;
    default:
        break;
    }
    return false;
}

// Load all variables from EEPROM
//
bool AP_Param::load_all(void)
{
//    float a;
    for (uint8_t vindex=0; vindex < _num_vars; vindex++){
        uint8_t type = PGM_UINT8(&_var_info[vindex].type);

        if (type == AP_PARAM_GROUP){
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[vindex].group_info);
			
			for (uint8_t i=0;
				 (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
				 i++){
				uintptr_t p = PGM_POINTER(&_var_info[vindex].ptr);
				AP_Param *ap = (AP_Param *)(p + PGM_POINTER(&group_info[i].offset));
				if (ap != NULL){
					ap->load();
//                    a = ap->cast_to_float((enum ap_var_type)group_info[i].type);
				}
			}
		}else{
            uintptr_t p = PGM_POINTER(&_var_info[vindex].ptr);
            AP_Param *ap = (AP_Param *)p;
            if (ap != NULL){
				ap->load();
//                a = ap->cast_to_float((enum ap_var_type)_var_info[vindex].type);
			}
        }
    }
    return true;
}

// Load all variables from EEPROM
//
bool AP_Param::save_all(void)
{
    for (uint8_t vindex=0; vindex<_num_vars; vindex++){
        uint8_t type = PGM_UINT8(&_var_info[vindex].type);

        if (type == AP_PARAM_GROUP){
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[vindex].group_info);
			
			for (uint8_t i=0;
				 (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
				 i++){
				uintptr_t p = PGM_POINTER(&_var_info[vindex].ptr);
				AP_Param *ap = (AP_Param *)(p + PGM_POINTER(&group_info[i].offset));
				if (ap != NULL){
					ap->save();
				}
			}
		}else{
            uintptr_t p = PGM_POINTER(&_var_info[vindex].ptr);
            AP_Param *ap = (AP_Param *)p;
            if (ap != NULL){
				ap->save();
			}
        }
    }
    return true;
}

// Load all variables from EEPROM
//
bool AP_Param::save_default_all(void)
{
    float value;
    for (uint8_t vindex=0; vindex<_num_vars; vindex++){
        uint8_t type = PGM_UINT8(&_var_info[vindex].type);

        if (type == AP_PARAM_GROUP){
            const struct GroupInfo *group_info = (const struct GroupInfo *)PGM_POINTER(&_var_info[vindex].group_info);
			
			for (uint8_t i=0;
				 (type=PGM_UINT8(&group_info[i].type)) != AP_PARAM_NONE;
				 i++){
				uintptr_t p = PGM_POINTER(&_var_info[vindex].ptr);
				AP_Param *ap = (AP_Param *)(p + PGM_POINTER(&group_info[i].offset));
				if (ap != NULL){
                    value = group_info[i].def_value;
                    set_value((enum ap_var_type)group_info[i].type, ap, &value);
					ap->save();
				}
			}
		}else{
            uintptr_t p = PGM_POINTER(&_var_info[vindex].ptr);
            AP_Param *ap = (AP_Param *)p;
            if (ap != NULL){
                value = _var_info[vindex].def_value;
                set_value((enum ap_var_type)_var_info[vindex].type, ap, &value);
				ap->save();
			}
        }
    }
    return true;
}
