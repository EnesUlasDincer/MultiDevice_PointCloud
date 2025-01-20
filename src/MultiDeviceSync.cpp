/*
Notes:
## on the Arm/Linux platform ,this sample requires users to compile with
Opencv4.2 or above,otherwise, it cannot be rendered.

## Increasing the USBFS buffer size

By default, the USBFS buffer size is 16 MB. This value is insufficient for
high-resolution images or multiple streams and multiple devices usage. User can
increase the buffer size to 128 MB.

### Check the USBFS buffer size:

```bash
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

### Increase the USBFS buffer size until the next reboot (here: example value
128)

```bash
sudo sh -c 'echo 128 > /sys/module/usbcore/parameters/usbfs_memory_mb'
```

### Increase the USBFS buffer size permanently

To increase the buffer size permanently, add the kernel parameter
usbcore.usbfs_memory_mb=128 to the bootloader configuration. How to do this
depends on the bootloader on your system.

For GRUB2 (most common):

1. Open /etc/default/grub. Replace: `GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"`
(or other contents within the quotation marks depending on your system) with:
`GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=128"`
2. Update grub

   ```bash
    sudo update-grub
   ```
3. reboot your system

Other bootloaders: configure additional kernel parameters of other bootloaders,
please see the manual of your bootloader.

*/

#include "../include/multiDevice_funcs.hpp"

int main(int argc, char **argv) {
  std::cout << "Please select options: " << std::endl;
  std::cout << " 0 --> config devices" << std::endl;
  std::cout << " 1 --> start stream" << std::endl;
  std::cout << "input: ";
  int index = -1;
  std::cin >> index;
  std::cout << std::endl;

  int exitValue = -1;
  if (index == 0) {
    exitValue = configMultiDeviceSync();
    // Only after the configuration is successful, the follow-up test is allowed
    // to continue
    if (exitValue == 0) {
      exitValue = testMultiDeviceSync();
    }
  } else if (index == 1) {
    exitValue = testMultiDeviceSync();
  } else {
    std::cout << "invalid index. " << std::endl;
    std::cout << "Please press any key to exit" << std::endl;
  }

  if (exitValue != 0) {
    wait_any_key();
  }

  return exitValue;
}

