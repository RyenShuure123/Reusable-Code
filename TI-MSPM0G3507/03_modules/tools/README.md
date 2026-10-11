# 小工具

- `open_sysconfig.bat`：以指定 SDK 为 product 打开 SysConfig 图形界面，
  可带一个 `.syscfg` 参数直接打开配置文件。路径按自己安装位置修改：

  ```bat
  open_sysconfig.bat                          # 空配置新建
  open_sysconfig.bat syscfg\xxx.syscfg        # 打开已有配置
  ```

01/02 工程各自的 `tools/` 里还有：OpenOCD 烧录脚本（DAPLink/XDS110）、
**MSPM0 工厂复位脚本**（NONMAIN 配置错误导致芯片锁死时的恢复手段，慎用，
会擦全片）、SysConfig CLI 生成脚本。
