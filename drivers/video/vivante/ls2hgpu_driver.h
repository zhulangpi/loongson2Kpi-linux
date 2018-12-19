int ls2hgpu_drv_open(
		    struct inode* inode,
		        struct file* filp
			    );

int ls2hgpu_drv_release(
		    struct inode* inode,
		        struct file* filp
			    );

long ls2hgpu_drv_ioctl(
		    struct file* filp,
		        unsigned int ioctlCode,
			    unsigned long arg
			        );

int ls2hgpu_drv_mmap(
		    struct file* filp,
		        struct vm_area_struct* vma
			    );

int ls2hgpu_gpu_probe(struct platform_device *pdev);

int ls2hgpu_gpu_remove(struct platform_device *pdev);

int ls2hgpu_gpu_suspend(struct platform_device *dev, pm_message_t state);

int ls2hgpu_gpu_resume(struct platform_device *dev);
