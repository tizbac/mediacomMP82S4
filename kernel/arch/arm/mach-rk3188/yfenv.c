#include <linux/kernel.h>
#include <linux/string.h>

//make sure it's not all zero
const char _board_env[0x1000] = "\003ver\0050.01";
static const char * s_es, * s_ie, * s_re;

static const char * env_get(char * name, const char * es, const char * ee)
{
	int nlen, vlen;
	const char * value;
	int len = strlen(name);
	while(es < ee) {
		nlen = (unsigned char)*es++;
		value = es + nlen;
		if(len == nlen && !memcmp(es, name, len)) {
			return value;
		}
		vlen = (unsigned char)*value++;
		es = value + vlen;
	}
	return 0;
}

const char * env_get_raw(char * name)
{
	return env_get(name, s_ie, s_re);
}

const char * env_get_str(char * name, const char * def)
{
	const char * value = env_get(name, s_ie, s_re);
	if(value && !value[*(unsigned char *)value]) {
		def = value + 1;
	}
	return def;
}

unsigned env_get_len(void * buffer)
{
	char * cb = (char *)buffer;
	if(cb > s_es && cb < s_re){
		return (unsigned char)cb[-1];
	}
	return 0;
}

unsigned * env_get_u32s(char *name)
{
	const char * value = env_get(name, s_es, s_ie);
	if(value) {
		int len = (unsigned char)*value++;
		if(len >= 4) {
			return (unsigned *)(value + (len & 3));
		}
	}
	return 0;
}

unsigned env_get_u32(char *name, unsigned value)
{
	unsigned * values = env_get_u32s(name);
	if(values) {
		value = *values;
	}
	return value;
}

unsigned env_cpy_u8s(char *name, void * buffer, unsigned len)
{
	unsigned rlen = 0;
	const char * value = env_get(name, s_ie, s_re);
	if(value) {
		rlen = (unsigned char)*value++;
		if(len == 0 || buffer == 0) {
			return rlen;
		}
		if(rlen > len) {
			rlen = len;
		}
		memcpy(buffer, value, rlen);
	}
	return rlen;
}

unsigned env_cpy_u32s(char *name, unsigned * buffer, unsigned len)
{
	unsigned rlen = 0;
	const char * value = env_get(name, s_es, s_ie);
	if(value) {
		rlen = (unsigned char)*value++;
		value += (rlen & 3);
		rlen >>= 2;
		if(len == 0 || buffer == 0) {
			return rlen;
		}
		if(rlen > len) {
			rlen = len;
		}
		memcpy(buffer, value, rlen << 2);
	}
	return rlen;
}

int env_enum_u32(char * name,void * param,int (*on_enum)(char *,unsigned *,void *))
{
	int count = 0;
	int nlen, vlen;
	char buffer[256];
	int len = strlen(name);
	const char * value, * es = s_es;
	while(es < s_ie) {
		nlen = (unsigned char)*es++;
		value = es + nlen;
		vlen = (unsigned char)*value++;
		if(nlen >= len && !memcmp(es, name, len)) {
			memcpy(buffer, es, nlen);
			buffer[nlen] = 0;
			value += vlen & 3;
			vlen &= ~3;
			count++;
			if(on_enum(buffer, (unsigned *)value, param)) {
				break;
			}
		}
		es = value + vlen;
	}
	return count;
}

int env_enum_str(char * name,void * param,int (*on_enum)(char *,char *,void *))
{
	int count = 0;
	int nlen, vlen;
	char buffer[256];
	int len = strlen(name);
	const char * value, * es = s_ie;
	while(es < s_re) {
		nlen = (unsigned char)*es++;
		value = es + nlen;
		vlen = (unsigned char)*value++;
		if(nlen >= len && !memcmp(es, name, len) && !value[vlen-1]) {
			memcpy(buffer, es, nlen);
			buffer[nlen] = 0;
			count++;
			if(on_enum(buffer, (char *)value, param)) {
				break;
			}
		}
		es = value + vlen;
	}
	return count;
}

void __init env_fixup(void)
{
	int elen;
	const char * end;
	if(s_es > s_re) {
		//never come here
		__asm__ __volatile__
		(
		".word 0x5F454E56\n"
		//".word _board_env - __init_begin\n"
		".word _board_env\n"
		".word __init_begin\n"
		);
	}
	end = _board_env + sizeof(_board_env);
	s_re = _board_env;
	s_es = _board_env;
	s_ie = 0;
	while(*s_re) {
		elen = (unsigned char)(*s_re);
		elen += 1;
		elen += (unsigned char)(s_re[elen]);
		elen += 1;
		if(s_re + elen >= end) {
			break;
		}
		if(s_ie == NULL && (elen & 3)) {
			s_ie = s_re;
		}
		s_re += elen;
	}
	if(s_ie == 0) {
		s_ie = s_re;
	}
}
