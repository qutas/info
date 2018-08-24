#!/usr/bin/python

from lxml import html
import requests

page = requests.get('http://192.168.1.1')
tree = html.fromstring(page.content)

wifi_script = tree.xpath('//head/script')
s = wifi_script[-1] #The script with wifi ids in is the script last in the head

wifi_clients = 0

for l in s.text.split("\n"):
	if "setWirelessTable('xx" in  l:
		#print(l)
		wifi_clients += 1

print("<txt>" + str(wifi_clients) + "</txt>")
print('<img>/usr/share/icons/Adwaita/scalable/devices/modem-symbolic.svg</img>')
print('<click>xdg-open http://192.168.1.1</click>')
