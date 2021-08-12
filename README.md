# Discrete-IR-Code-implant
A means of providing Humax YouView box with discrete Power On and Power Off Commands
A 'standard problem' with Infra Red (I/R) control consumer AV gear is that some vendors do not provide discrete Power On and Power Off commands - only a toggle.  That makes automation (e.g. RTI, Crestron, Pronto) impossible to make reliable, as you can't know for sure whether the device is on or off.
Some brands - e.g. Sky, and many televisions - will always turn on if you press a particular code.
Others - e.g. Humax - just don't have ANY way to be sure whether they are on or off.

This project is about solving that problem for a Humax.  If you are sufficiently 'up' on writing code for PICs, the same principle could doubtless be used with other devices.

**WARNING**  If you are not confident about opening up your device, discovering the points to attach the four connections to, or are worried about voiding warranty, read no further.  This project is not for you.

HOW IT WORKS:
In almost all cases, I/R controlled AV equipment use a three-pin I/R receiver module (such as the Vishay TSOP31238) which has Ground, +5V and Signal connections.
The strategy is to connect two microcontroller pins to the equipment as follows
* One to some point in the equipment which can monitor its state - i.e. on or off (more below)
* Another to the Signal connection so we can monitor incoming I/R commands, and to inject the 'Power Toggle' command when appropriate

We also need connections to the power supply and Ground, both of which we can pick up from the back of the I/R receiver module - four wires in total.



The strategy therefore is to identify at least one unused I/R codes for the device in question - which we call 'Power Off'.
You can either then identify a second unused code and call it 'Power On', OR choose to monitor a code such as 'OK' or 'Menu' which you use as a trigger to power the box on.  If you do that, choose a code which won't cause mayhem if it is sent to the equipment when it is powered on.

In my case I chose to have the box switch on when the 'YouView' button was pressed on the remote - but be careful - this is a button which toggles the top level menu - and the box retains the state of menu presentation even when turned "off", so for my remote programming, I use a second unused code.

We then program the microcontroller to recognise those codes, and act as follows:
- If we receive the Power On (or chosen monitored) command, and the equipment is "off", inject a Power Toggle command
- If we receive the Power Off command, and the equipment is "on", inject a Power Toggle command
- Otherwise, do nothing - the equipment is already  in the desired state.

The pdf document 'Connecting to a Humax YouView' says much more about how this can be done.
The pdf document 'Implant implementation' explains how I made mine and contains a few hints and tips.
The 'C' file is the firmware for the microcontroller which can be pulled into MPLAB-X

So if your appetite has been whetted - read on!
