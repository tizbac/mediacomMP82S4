/*
 * Copyright (C) 2011 YuanFeng Technology
 * liqiangman@yftech.com
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define MAX_VALUE_NUM 256
#define MAX_MACRO_NUM 16
#define MAX_MACRO_VAR 8
#define MODE_REPLACE  0
#define MODE_DELETE   1
#define MODE_APPEND   2

#define TYPE_INT      0
#define TYPE_CHAR     1
#define TYPE_STR      2

static char * s_fs = NULL;
static char * s_es = NULL;
static char * s_em;
static char * s_fe, * s_ee, * s_ie, * s_re; //end of file, env, int env, raw env
static char * s_input = NULL;
static char * s_output = NULL;
static unsigned s_env_start = 0;
static unsigned s_env_step = 4;
static unsigned s_env_size = 0;
static unsigned s_env_entry = 0;
static char * s_macro_name[MAX_MACRO_NUM+1];
static unsigned s_macro_vars[MAX_MACRO_NUM][MAX_MACRO_VAR+1];
static int s_mode = MODE_REPLACE;

static void trim_back(char * buffer)
{
	do {
		*buffer-- = 0;
	}
	while(*buffer == ' ' || *buffer == '\t');
}
static int parse_macro(char * str, char **end, unsigned * pvalue)
{
	int i = 0;
	char cm, c, * macro, *temp;
	unsigned value, multi, *vars;
	while(*str == ' ' || *str == '\t') str++;
	while((macro = s_macro_name[i])) {
		vars = s_macro_vars[i];
		value = *++vars; //bpp ingored
		temp = str;
		while((cm = *macro++)) {
			if(cm == '%') {
				multi = *++vars;
				c = *macro++;
				if(c == 'c') {
					c = *temp++;
					if(c >= 'a' && c <= 'z') {
						value += (c - 'a') * multi;
					}
					else {
						break;
					}
				}
				else if(c == 'C') {
					c = *temp++;
					if(c >= 'A' && c <= 'Z') {
						value += (c - 'A') * multi;
					}
					else {
						break;
					}
				}
				else { //assume d
					value += strtol(temp, end, 0) * multi;
					if(*end == temp) {
						break;
					}
					temp = *end;
				}
			}
			else if(cm != *temp++) {
				break;
			}
		}
		if(cm == 0) {
			if(*temp == 0 || *temp == ' ' || *temp == '\t' || *temp == ',') {
				*pvalue = value;
				*end = temp;
				return 1;
			}
		}
		i++;
	}
	return 0;
}
static unsigned parse_ints(char * str, unsigned * values, unsigned max, int macro)
{
	char * end;
	unsigned num = 0, bpp = 1, value;
	while(*str && num < max) {
		value = strtoul(str, &end, 0);
		if(end == str) {
			if(macro == 0 || parse_macro(str, &end, &value) == 0) {
				return 0;
			}
		}
		if(bpp == 1) {
			//only 0xN, 0xNN treat as char
			if(end - str < 3 || (end[-3] != 'x' && end[-3] != 'X' && 
			                      end[-2] != 'x' && end[-2] != 'X')) {
				bpp = 4;
			}
		}
		str = end;
		values[++num] = value;
		while(*str == ' ' || *str == '\t') str++;
		if(*str == ',') str++;
	}
	*values = bpp;
	return num;
}
static int parse_int(char * str, unsigned * pvalue)
{
	char * end;
	unsigned value = strtoul(str, &end, 0);
	if(end != str) {
		*pvalue = value;
		return 1;
	}
	return 0;
}
static char * open_file(char * name, unsigned * psize)
{
	unsigned size = 0;
	char * buffer = NULL;
	FILE *file = fopen(name, "rb");
	if(file) {
		fseek(file, 0, SEEK_END);
		size= ftell(file);
		if(size && (buffer = (char *)malloc(size))) {
			fseek(file, 0, SEEK_SET);
			fread(buffer, 1, size, file);
		}
		fclose(file);
	}
	else {
		printf("failed to open input file '%s'\n", name);
	}
	*psize = size;
	return buffer;
}
static void save_file(char * name)
{
	FILE *file;
	if(name == NULL) {
		return;
	}
	if(s_fs == NULL || s_env_entry == 0) {
		printf("nothing to save\n");
		return;
	}
	file = fopen(name, "wb");
	if(file) {
		s_env_entry = 0;
		memset(s_re, 0, s_em - s_re + 1);
		fwrite(s_fs, 1, s_fe - s_fs, file);
		fclose(file);
		printf("save file '%s'\n", name);
	}
	else {
		printf("failed to open output file '%s'\n", name);
	}
}

static char * insert_env(char * inserted, int length)
{
	if(length == 0) return inserted; //no need to memmove
	if(length + s_re >= s_ee) return NULL;
	if(s_re != inserted) {
		if(length < 0) {
			inserted -= length;
		}
		memmove(inserted+length, inserted, s_re-inserted);
	}
	if(s_ie >= inserted) s_ie += length;
	s_re += length;
	if(s_em < s_re) s_em = s_re;
	return inserted;
}

static char * replace_env(char * replaced, int length)
{
	char * env = replaced;
	int old_len = *env++;
	env += old_len;
	old_len += 2 + *env++;
	return insert_env(replaced, length - old_len);
}

static void handle_entry(char * name, void * value, size_t length, int type)
{
	size_t olen, vlen, number = 0;
	size_t nlen = strlen(name);
	int deleted = s_mode == MODE_DELETE;
	char * start, * end, * ntemp, *vtemp;
	if(type) {
		start = s_ie;
		end = s_re;
	}
	else {
		start = s_es;
		if(deleted) {
			length = 0;
			end = s_re;
			number = *(size_t *)value;
		}
		else {
			end = s_ie;
		}
	}
	if(s_mode == MODE_APPEND) {
		start = end;
	}
	
	while(start < end) {
		ntemp = start;
		olen = (unsigned char)(*ntemp++);
		vtemp = ntemp + olen;
		vlen = (unsigned char)(*vtemp++);
		if(olen == nlen && !memcmp(name, ntemp, nlen)) { //name match
			if(deleted) {
				if(length) {
					if(vlen >= length && !memcmp(vtemp, value, length)) { //value match
						replace_env(start, 0);
						deleted++;
						end -= ((vtemp-start) + vlen);
						continue;
					}
				}
				else if(number--) {
					replace_env(start, 0);
					deleted++;
					end -= ((vtemp-start) + vlen);
					if(number == 0) {
						break;
					}
					continue;
				}
			}
			else {
				break;
			}
		}
		start = vtemp + vlen;
	}
	if(deleted--) {
		printf("%d entry deleted for name '%s'\n", deleted, name);
		if(deleted) {
			s_env_entry++;
		}
	}
	else {
		if(type == TYPE_STR) {
			length ++;
		}
		else if(type == TYPE_INT) {
			number = 3 - ((nlen + 1) & 3) ;
		}
		vlen = nlen + 2 + number + length; //entry length
		if(start < end) {
			start = replace_env(start, vlen);
		}
		else {
			start = insert_env(start, vlen);
		}
		if(start) {
			*start++ = nlen;
			memcpy(start, name, nlen);
			start += nlen;
			do {
				*start++ = length+number;
			}
			while(number--);
			memcpy(start, value, length);
			s_env_entry++;
		}
		else {
			printf("failed to handle entry '%s'\n", name);
		}
	}
}

static void set_env(char * name, char * value)
{
	unsigned number;
	unsigned values[MAX_VALUE_NUM + 1];
	if(s_es == NULL) {
		printf("ingore env '%s' = '%s'\n", name, value);
		return;
	}
	if((number = parse_ints(value, values, MAX_VALUE_NUM, 1))) {
		char * format;
		unsigned * uvalues = values + 1;
		if(*values == 1) {
			unsigned i;
			for(i = 0; i < number; i++) {
				value[i] = (char)uvalues[i];
			}
			handle_entry(name, value, number, TYPE_CHAR);
			format = "%02x ";
		}
		else {
			handle_entry(name, uvalues, number * 4, TYPE_INT);
			format = "%08x ";
		}
		printf("'%s'=", name);
		while(number--) {
			printf(format, *uvalues++);
		}
		printf("\n");
	}
	else {
		printf("'%s'='%s'\n", name, value);
		handle_entry(name, value, strlen(value), TYPE_STR);
	}
}
static void set_magic(char * value)
{
	char * env;
	unsigned magic;
	s_es = NULL;
	if(parse_int(value, &magic) == 0) {
		printf("invalid magic number %s\n", value);
		return;
	}
	if(s_fs == NULL) {
		printf("input must be set before env_magic\n");
		return;
	}
	printf("search for env, start 0x%x, step %d, magic 0x%x\n", s_env_start, s_env_step, magic);
	for(env = s_fs + s_env_start; env < s_fe-8; env += s_env_step) {
		if(*(unsigned *)env == magic) {
			s_es = s_fs + *(unsigned *)(env + 4) - *(unsigned *)(env + 8);
			if(s_es < s_fe) {
				int elen;
				if(s_env_size == 0) {
					s_ee = s_fe;
				}
				else {
					s_ee = s_es + s_env_size;
					if(s_ee > s_fe) {
						s_ee = s_fe;
					}
				}
				s_ie = NULL;
				s_re = s_es;
				//move to end of original env
				while(*s_re) {
					elen = (unsigned char)(*s_re);
					elen += 1;
					elen += (unsigned char)(s_re[elen]);
					elen += 1;
					if(s_re + elen >= s_ee) {
						break;
					}
					if(s_ie == NULL && (elen & 3)) {
						s_ie = s_re;
					}
					s_re += elen;
				}
				if(s_ie == NULL) {
					s_ie = s_re;
				}
				s_em = s_re;

				//clear variable for new magic
				s_mode = MODE_REPLACE;
				s_macro_name[0] = NULL;

				printf("env found at offset 0x%lx, size %ld\n", s_es - s_fs, s_ee - s_es);
				return;
			}
			s_es = NULL;
		}
	}
	printf("env not found\n");
}
static void set_macro(char * name, char * value)
{
	int i;
	unsigned num;
	for(i = 0; i < MAX_MACRO_NUM; i++) {
		if(s_macro_name[i] == NULL) {
			num = parse_ints(value, s_macro_vars[i], MAX_MACRO_VAR, 0);
			if(num) {
				char * temp = name;
				while(*temp) {
					if(*temp++ == '%') num--;
				}
				if(num == 1) {
					s_macro_name[i] = name;
					printf("macro '%s' %d 0x%x\n", name, s_macro_vars[i][0], s_macro_vars[i][1]);
					return;
				}
			}
			break;
		}
	}
	printf("ingore macro '%s' = '%s'\n", name, value);
}
static void set_comamnd(char * name, char * value)
{
	if(!strcmp(name, "input")) {
		unsigned size;
		if(s_fs) {
			free(s_fs);
		}
		s_fs = open_file(value, &size);
		if(s_fs) {
			s_fe = s_fs + size;
			s_input = value;
		}
		else {
			s_input = NULL;
		}
	}
	else if(!strcmp(name, "output")) {
		if(s_output) {
			save_file(s_output);
		}
		s_output = value;
	}
	else if(!strcmp(name, "env_start")) {
		parse_int(value, &s_env_start);
	}
	else if(!strcmp(name, "env_step")) {
		parse_int(value, &s_env_step);
	}
	else if(!strcmp(name, "env_size")) {
		parse_int(value, &s_env_size);
	}
	else if(!strcmp(name, "env_magic")) { //must be last param
		set_magic(value);
	}
	else if(!strcmp(name, "mode")) {
		if(!strcmp(value, "delete")) s_mode = MODE_DELETE;
		else if(!strcmp(value, "append")) s_mode = MODE_APPEND;
		else if(!strcmp(value, "replace")) s_mode = MODE_REPLACE;
		else if(!strcmp(value, "clear")) {
			printf("clear all\n");
			s_ie = s_re = s_es;
			set_env("ver", "0.02");
			s_ie = s_es;
		}
		else {
			printf("unknown mode '%s'\n", value);
			return;
		}
		printf("change to %s mode\n", value);
	}
	else {
		set_macro(name, value);
	}
}
static void parse_env(char * buffer, char * end)
{
	int c;
	int second = 0,comment = 0, command = 0;
	char * name = 0,* value = 0;
	while(buffer < end) {
		c = *buffer;
		if(c == '\r' || c == '\n') {
			trim_back(buffer);
			if(name && value) {
				if(command) {
					set_comamnd(name, value);
				}
				else {
					set_env(name,value);
				}
			}
			name = value = 0;
			second = comment = command = 0;
		}
		else if(c == ';') {
			if(name == NULL && buffer[1] == '@') {
				buffer++;
				command = 1;
			}
			else {
				trim_back(buffer);
				comment = 1;
			}
		}
		else if(c == '=') {
			trim_back(buffer);
			second = 1;
		}
		else if(!comment && c != ' ' && c != '\t') {
			if(c == '\\') {
				int skip = 1;
				if(buffer[skip] == '\r') skip++;
				if(buffer[skip] == '\n') skip++;
				if(skip > 1) {
					if(name) {
						memcpy(name + skip , name, buffer - name);
						name += skip;
						if(value) {
							value += skip;
						}
					}
					buffer += skip;
					continue;
				}
			}
			if(second) {
				value = buffer;
				second = 0;
			}
			else if(name == 0) {
				name = buffer;
			}
		}
		buffer++;
	}
}
int main(int argc , char * argv[])
{
	char * buffer;
	char * config = "config.txt";
	unsigned size;
	if(argc > 1) {
		config = argv[1];
		if(argc > 2) {
			set_comamnd("input", argv[2]);
			if(argc > 3) {
				set_comamnd("output", argv[3]);
			}
		}
	}
	buffer = open_file(config, &size);
	if(buffer) {
		parse_env(buffer, buffer + size);
		save_file(s_output ? s_output : s_input);
		free(buffer);
	}
	if(s_fs) {
		free(s_fs);
	}
	return 0;
}
