板子上运行：
（注意192.168.27.205是主机的ip,而且随着代理节点变而变。注意电脑不能联校园网，只能联手机热点）
ssh -L 7899:localhost:7890 user-888@192.168.27.205 -N -f

代理延续到容器内：
docker run -it --net=host \
  -v /mnt/data/kairui.wang/test:/mnt/test \
  -e http_proxy="http://127.0.0.1:7899" \
  -e https_proxy="http://127.0.0.1:7899" \
  -e no_proxy="localhost,127.0.0.1,*.local,192.168.27.0/24" \
  --entrypoint="/bin/bash" \
  e42755fdcb63

测试：
curl -v http://www.google.com