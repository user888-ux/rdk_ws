# 1.打开https://2cy.io/user，右下角“订阅导入（不推荐）”处鼠标悬停在clash订阅，然后选择第二项“复制clash订阅”

# 2.输入命令
clashsub add “刚刚复制的”
一般会提示：
⏳ 正在下载...
🍃 验证订阅配置...
🎉 订阅已添加：[num] https://45.137.181.221/link/pZLplYRS5p7Op9Q7?clash=1

# 3.输入命令
clashsub update num(就是上面“订阅已添加后面那个数”)
一般提示：
✈️  更新订阅：[3] https://45.137.181.221/link/pZLplYRS5p7Op9Q7?clash=1
⏳ 正在下载...
🍃 验证订阅配置...
😼 订阅已更新

# 4.输入命令
clashsub use 3
一般输出：
🔥 订阅已生效

# 5.输入命令
pkill clash
nohup ./clash -d ~/.config/clash > clash.log 2>&1 &  # 后台运行
clashon

# 6.进入http://localhost:9090/ui/#/proxies并在Global（或Proxies）下选择其中一个
注：输入clashctl ui得到放行端口9090

# 7.测试连通性
curl --proxy http://127.0.0.1:7890 https://www.google.com --connect-timeout 10
要输出一堆xml段才算成功，如：
<!doctype html><html itemscope="" itemtype="http://schema.org/WebPage" lang="zh-TW"><head><meta content="text/html; charset=UTF-8" http-equiv="Content-Type"><meta content="/images/branding/googleg/1x/googleg_standard_color_128dp.png" itemprop="image"><title>Google</title><script nonce="XzaVPpH15etZjzmT9IGYIA">(function......

curl -x http://127.0.0.1:7890 https://registry-1.docker.io/v2/
一般输出：{"errors":[{"code":"UNAUTHORIZED","message":"authentication required","detail":null}]} 为成功

注：输入clashctl proxy获得系统代理的端口7890

# 8.如果想在Firefox上访问
先下载浏览器插件Zero Omega,然后点击右上角的圈，点击选项，配置Proxy
全部设置为Http和对应端口（这里是7890），然后再次点击圈，选择proxy

# 9.命令行帮助
user-888@user888-Lenovo-G5000-IRX9:~/clash-for-linux-install$ clashctl -h
    
Usage: 
  clashctl COMMAND [OPTIONS]

Commands:
  on                    开启代理
  off                   关闭代理
  proxy                 系统代理
  status                内核状态
  ui                    面板地址
  sub                   订阅管理
  log                   内核日志
  tun                   Tun 模式
  mixin                 Mixin 配置
  secret                Web 密钥
  upgrade               升级内核

Global Options:
  -h, --help            显示帮助信息

For more help on how to use clashctl, head to https://github.com/nelvko/clash-for-linux-install

# 2026-2-10新增
# 10.注意如果失效就尝试在 http://localhost:9090/ui/#/proxies 更换节点，然后测试连通性

# 2026-2-28新增
# 11.如果权限不够：
sudo chown user-888:user-888 代码的git地址
# 配置git代理
git config --global http.proxy https://127.0.0.1:7890
git config --global https.proxy https://127.0.0.1:7890 （代理服务器地址）
# 报错：
fatal: 无法访问 'https://github.com/D-Robotics/robot_dev_config.git/'：GnuTLS recv error (-110): The TLS connection was non-properly terminated.
方法：将git前面的sudo去掉，然后用上面的方法加权限
