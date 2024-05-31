# HCI-Adaptor

## 应用说明
HCI Adaptor Demo可应用于DTM示例，适配第三方Host（如Zephyer Host Stack、Nimble Host Stack等），所展示的Demo演示了：

* GR5xxx Controller Stack和Nimble Host Stack运行在同一芯片上，完成CSCS Profile应用的演示。（ble_app_ext_host_combine）
* GR5xxx Controller Stack和Nimble Host Stack分别运行在独立芯片上，之前通过串口通信，完成CSCS Profile应用的演示。（ble_app_ext_host_only & ble_app_int_controller_only）

## HCI适配
基于GR5xxx平台支持的H4TL协议，提供有Host to Controller和Controller to Host数据发送接口，以及用于判断是否需要流程的Buffer剩余空间获取的接口

## 运行展示

* 先clone GR533x SDK到本地工作区, 
* 将Demo工程拷贝到 ${GR533x.SDK}\projects\ble\ble_peripheral 目录下
* 使用 keil环境构建编译下载到 GR533x SK开发板体验即可
* 如果有Goodix其他的SK板, 可以轻松移植到对应的SDK下

## 注意事项
* 运行于其他GR5xxx平台，需更换对应平台的custom_config.h，并使能CFG_CONTROLLER_ONLY
* 运行于其他GR5xxx平台存在编译错误，稍微对应修改即可，不会太复杂，自行修改
* 对于GR551x平台需将vhci_patch_for_gr551x中文件包含编译