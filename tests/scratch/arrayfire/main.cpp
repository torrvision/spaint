#include <arrayfire.h>

int main(int argc, char** argv)
{
	char t_device_name[64] = { 0 };
	char t_device_platform[64] = { 0 };
	char t_device_toolkit[64] = { 0 };
	char t_device_compute[64] = { 0 };
	af::deviceInfo(t_device_name, t_device_platform, t_device_toolkit, t_device_compute);

	printf("Device name: %s\n", t_device_name);
	printf("Platform name: %s\n", t_device_platform);
	printf("Toolkit: %s\n", t_device_toolkit);
	printf("Compute version: %s\n", t_device_compute);

	return 0;
}
