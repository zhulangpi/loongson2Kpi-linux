#include <south-bridge.h>

void __init plat_swiotlb_setup(void)
{
	if (ls_south_bridge->sb_init_swiotlb)
		ls_south_bridge->sb_init_swiotlb();
}
