#include <asm/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/random.h>
#include <linux/crypto.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <mach/yfmach.h>

#define MAX_DIGEST_SIZE              20
//#define dbg(...)
#define dbg printk

#define VER_IOC_MAGIC                'V'

#define VER_GET_ID                   0
#define VER_GET_VER                  1
#define VER_SET_VER                  2
#define VER_GET_INFO                 3
#define VER_SET_INFO                 4

static int ver_opened = 0;
static unsigned char * ver_hash;
static unsigned char * ver_key;

//===================== system data ===================//
#include <linux/syscalls.h>
#define RKNAND_GET_VENDOR_SECTOR0   _IOW('v', 16, unsigned int)
#define RKNAND_STORE_VENDOR_SECTOR0 _IOW('v', 17, unsigned int)
#define RKNAND_GET_VENDOR_SECTOR1   _IOW('v', 18, unsigned int)
#define RKNAND_STORE_VENDOR_SECTOR1 _IOW('v', 19, unsigned int)
#define RKNAND_SYS_STORGAE_DATA_LEN 512
#define VENDOR_SECTOR_OP_TAG        0x444E4556 // "VEND"

#define ERROR printk
typedef unsigned int uint32;
typedef unsigned char uint8;

typedef struct tagRKNAND_SYS_STORGAE
{
	uint32  tag;
	uint32  len;
	uint8   data[RKNAND_SYS_STORGAE_DATA_LEN];
}RKNAND_SYS_STORGAE;

static RKNAND_SYS_STORGAE sys_vendor0;
static RKNAND_SYS_STORGAE sys_vendor1;
static int sys_data_ready;
static DEFINE_MUTEX(sys_data_lock);

int sys_data_io(int cmd, void * arg)
{
	int ret;
	int sys_fd, old_fs;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	sys_fd = sys_open("/dev/rknand_sys_storage",O_RDWR,0);
	if(sys_fd < 0){
		ERROR("rknand_sys_storage open fail %d\n", sys_fd);
		ret = sys_fd;
	}
	else {
		ret = sys_ioctl(sys_fd, cmd, (long)arg);
		if(ret){
			ERROR("get vendor_sector error for %08x\n", cmd);
		}
	}
	sys_close(sys_fd);
	set_fs(old_fs);
	return 0;
}

static int sys_data_init(void)
{
	if(sys_data_ready) return 0;
	sys_vendor0.tag = VENDOR_SECTOR_OP_TAG;
	sys_vendor0.len = RKNAND_SYS_STORGAE_DATA_LEN-8;
	if(sys_data_io(RKNAND_GET_VENDOR_SECTOR0, &sys_vendor0)) {
		printk("failed to init sys sector 0\n");
		return -EIO;
	}
	sys_vendor1.tag = VENDOR_SECTOR_OP_TAG;
	sys_vendor1.len = RKNAND_SYS_STORGAE_DATA_LEN-8;
	if(sys_data_io(RKNAND_GET_VENDOR_SECTOR1, &sys_vendor1)) {
		printk("failed to init sys sector 1\n");
		return -EIO;
	}
	sys_data_ready = 1;
	printk("init sys sector ok\n");
	return 0;
}

int sys_data_read(int index, int size, void * buf)
{
	int ret;
	if(index & SYS_DATA_MARK) {
		index = 5+(index & SYS_DATA_MASK);
	}
	if(index > 8 || size <= 0 || size > 62 || !buf) {
		return -EINVAL;
	}
	mutex_lock(&sys_data_lock);
	ret = sys_data_init();
	if(ret == 0) {
		uint8 * src = index ? sys_vendor1.data : sys_vendor0.data;
		if(index) src += (index -1)*62;
		memcpy(buf, src, size);
		ret = size;
	}
	mutex_unlock(&sys_data_lock);
	return ret;
}
int sys_data_write(int index, int size, void * buf)
{
	int ret;
	if(index & SYS_DATA_MARK) {
		index = 5+(index & SYS_DATA_MASK);
	}
	if(index > 8 || size <= 0 || size > 62 || !buf) {
		return -EINVAL;
	}
	mutex_lock(&sys_data_lock);
	ret = sys_data_init();
	if(ret == 0) {
		uint8 tmp[62];
		uint8 * src;
		int cmd;
		void * sys;
		if(index) {
			cmd = RKNAND_STORE_VENDOR_SECTOR1;
			src = sys_vendor1.data + (index -1)*62;
			sys = &sys_vendor1;
		}
		else {
			cmd = RKNAND_STORE_VENDOR_SECTOR0;
			src = sys_vendor0.data;
			sys = &sys_vendor0;
		}
		memcpy(tmp, src, size);
		memcpy(src, buf, size);
		ret = sys_data_io(cmd, sys);
		if(ret) {
			memcpy(src, tmp, size);
			ret = -EIO;
		}
		else {
			ret = size;
		}
	}
	mutex_unlock(&sys_data_lock);
	return ret;
}
//===================== system data ===================//

static int ver_verify(void)
{
	return 1;
}

static int ver_translate(void * message, int size)
{
	struct scatterlist sg[2];
	struct crypto_hash *tfm;
	struct hash_desc desc;
	int ret;

	tfm = crypto_alloc_hash("sha1", 0, CRYPTO_ALG_ASYNC);
	if (IS_ERR(tfm)) {
		return PTR_ERR(tfm);
	}

	desc.tfm = tfm;
	desc.flags = 0;
	sg_init_table(sg, 2);
	sg_set_buf(&sg[0], message, size);
	sg_set_buf(&sg[1], ver_verify() ? ver_key : ver_hash, 16);

	ret = crypto_hash_digest(&desc, sg, size+16, ver_hash);

	crypto_free_hash(tfm);
	return ret;
}

static unsigned char ver_get_crc(unsigned char * buffer, int length)
{
	int crc = 0;
	while(length--) {
		crc += *buffer++;
	}
	return (unsigned char)crc;
}

#define MAX_CUSTOM_SIZE  60
#define MAX_CUSTOM_INDEX 5
static int ver_custom_info(int cmd, int size, void * arg)
{
	int index = cmd & 0xF;
	int result = -EIO;
	unsigned char buffer[MAX_CUSTOM_SIZE+2];
	cmd >>= 4;
	if(index >= MAX_CUSTOM_INDEX) {
		printk("set info,too big index %d\n", index);
		return -EINVAL;
	}
	if((cmd & 7) == VER_SET_INFO) {
		if(size > MAX_CUSTOM_SIZE) size = MAX_CUSTOM_SIZE;
		if(!copy_from_user(buffer+2, arg, size)) {
			buffer[0] = size;
			buffer[1] = ver_get_crc(buffer+2, size);
			size = sys_data_write(index, size+2, buffer);
			if(size > 2) {
				result = size -2;
			}
		}
	}
	else {
		if(sys_data_read(index, sizeof(buffer), buffer) > 2) {
			int rsize = buffer[0];
			int crc = buffer[1];
			if(rsize <= MAX_CUSTOM_SIZE && crc == ver_get_crc(buffer+2, rsize)) {
				if(size > rsize) size = rsize;
				if(!copy_to_user(arg, buffer+2, size)) {
					result = size;
				}
			}
		}
	}
	dbg("%s cmd 0x%x result %d\n",__FUNCTION__, cmd, result);
	return result;
}

#ifdef CONFIG_PM
static int ver_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int ver_resume(struct platform_device *dev)
{
	return 0;
}
#else
#define ver_suspend NULL
#define ver_resume NULL
#endif

static long ver_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void * buffer;
	int ret = -EFAULT;
	int size = _IOC_SIZE(cmd);

	dbg("%s cmd 0x%x\n",__FUNCTION__, cmd);

	if(_IOC_TYPE(cmd) != VER_IOC_MAGIC)	{
		return -ENOTTY;
	}
	if(_IOC_DIR(cmd) & _IOC_READ) {
		if(!access_ok(VERIFY_WRITE, (void __user*)arg, size)) {
			return -EFAULT;
		}
	}
	else if(!access_ok(VERIFY_READ, (void __user*)arg, size)) {
		return -EFAULT;
	}
	switch(_IOC_NR(cmd) >> 4) {
		case VER_GET_ID:
			if(size >= 6) {
				char id[6] = {0};
				if(copy_to_user((void *)arg, id, 6)) {
					ret = -EIO;
				}
				else {
					ret = 6;
				}
			}
			break;
		case VER_GET_VER:
			if(size >= MAX_DIGEST_SIZE) {
				if(copy_to_user((void *)arg, ver_hash, MAX_DIGEST_SIZE)) {
					ret = -EIO;
				}
				else {
					ret = MAX_DIGEST_SIZE;
				}
			}
			break;
		case VER_SET_VER:
			if(size && (buffer = kmalloc(size, GFP_KERNEL))) {
				if(!copy_from_user(buffer, (void *)arg, size)) {
					ret = ver_translate(buffer, size);
				}
				kfree(buffer);
			}
			break;
		case VER_GET_INFO:
		case VER_SET_INFO:
			ret = ver_custom_info(_IOC_NR(cmd), size, (void *)arg);
			break;
	}
	return ret;
}

static int ver_open(struct inode *inode, struct file *file)
{
	dbg("%s\n",__FUNCTION__);
	if(ver_opened) return -EAGAIN;
	ver_opened = 1;
	return 0;
}

static int ver_close(struct inode *inode, struct file *file)
{
	dbg("%s\n",__FUNCTION__);
	ver_opened = 0;
	return 0;
}

static const struct file_operations ver_fops = {
	.owner = THIS_MODULE,
	.open = ver_open,
	.unlocked_ioctl = ver_ioctl,
	.release = ver_close,
};

static struct miscdevice misc_ver_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ver",
	.fops = &ver_fops,
};

static int ver_probe(struct platform_device *dev)
{
	int err = -ENOMEM;
	ver_hash = kmalloc(MAX_DIGEST_SIZE + 16, GFP_KERNEL);
	if(ver_hash) {
		char key[] = {
			0x98, 0x48, 0x16, 0xfd, 0x32, 0x96, 0x22, 0x87,
			0x6e, 0x14, 0x90, 0x76, 0x34, 0x26, 0x4e, 0x6f,
			0x33, 0x2e, 0x9f, 0xb3, 0x38, 0x4f, 0x6c, 0x2b,
			0x98, 0x48, 0x16, 0xfd, 0x32, 0x96, 0x22, 0x87,
			0x6e, 0x14, 0x90, 0x76, 0x34, 0x26, 0x4e, 0x6f,
			0x33, 0x2e, 0x9f, 0xb3, 0x38, 0x4f, 0x6c, 0x2b,
		};
		memcpy(ver_hash, key, MAX_DIGEST_SIZE + 16);
		err = misc_register(&misc_ver_device);
		ver_key = ver_hash + MAX_DIGEST_SIZE;
	}
	if(err) {
		printk("ver_probe error %d\n", err);
		if(ver_hash) {
			kfree(ver_hash);
		}
	}
	return err;
}

static int __devexit ver_remove(struct platform_device *dev)
{
	misc_deregister(&misc_ver_device);
	kfree(ver_hash);
	return 0;
}

static struct platform_driver ver_driver = {
	.probe		= ver_probe,
	.remove		= __devexit_p(ver_remove),
	.suspend	= ver_suspend,
	.resume		= ver_resume,
	.driver		= {
		.name	= "ver",
		.owner	= THIS_MODULE,
	},
};

static struct platform_device ver_device = 
{
	.name = "ver",
};

int __init ver_init(void)
{
	platform_device_register(&ver_device);
	platform_driver_register(&ver_driver);
	return 0;
}

static void __exit ver_exit(void)
{
	platform_device_unregister(&ver_device);
	platform_driver_unregister(&ver_driver);
}
module_init(ver_init);
module_exit(ver_exit);
