#!/bin/bash

<<DESCRIPTION
    copy-config2user is a shell command that copies one's desktop configuration files 
    to another specified user.

    Usage: copy-config2user USERNAME
DESCRIPTION

<<COPYRIGHT
    Copyright (C) 2012  Serge YMR Stroobandt

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
COPYRIGHT

<<CONTACT
    See user on4aa on https://forum.xfce.org/viewtopic.php?id=4168
CONTACT

TOUSER=$1

function copyitem() {
	ITEM=$1

	echo; echo "Deleting $ITEM of user $TOUSER..."
	sudo rm -Rv /home/$TOUSER/$ITEM

	echo; echo "Copying $ITEM of user $USER to user $TOUSER..."
	sudo cp -av /home/$USER/$ITEM /home/$TOUSER/$ITEM
	sudo chown -R $TOUSER:$TOUSER /home/$TOUSER/$ITEM
}

if [ -z $1 ] || ! [ -d /home/$TOUSER ] || [ $USER = $TOUSER ]; then
	echo "Please, specify the user who will receive your desktop configuration."
else
	copyitem ".config/autostart/"
	copyitem ".config/Terminal/"
	copyitem ".config/Thunar/"
	copyitem ".config/xfce4/"
fi