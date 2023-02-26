import sigrokdecode as srd
import os
from common.srdhelper import SrdIntEnum

Pin = SrdIntEnum.from_str('Pin', 'RST CLK DAT')
Ann = SrdIntEnum.from_str('Ann', 'START CMD BITS')
State = SrdIntEnum.from_str('State','CMD DATA ACK')
cmdValues= {
    0x04:	"READ_UID",
    0x0b:	"READ_CID",
    0x0c:	"READ_DEVICE_ID",
    0x00:	"READ_FLASH",
    0x21:	"WRITE_FLASH",
    0x26:	"MASS_ERASE",
    0x22:	"PAGE_ERASE",
}

class Decoder(srd.Decoder):
	api_version = 3
	id = 'nuvo_icp'
	name = 'Nuvoton ICP'
	longname = 'Nuvoton ICP'
	desc = 'Nuvoton ICP'
	license = 'gplv2+'
	inputs = ['logic']
	outputs = []
	tags = ['Embedded/industrial']
	channels = (
		{'id': 'rst', 'name': 'RST', 'desc': 'Reset'},
		{'id': 'clk', 'name': 'CLK', 'desc': 'Clock'},
		{'id': 'dat', 'name': 'DAT', 'desc': 'Data'},
	)
	optional_channels = ()
	annotations = (
		('s', 'Start sequence'),
		('c', 'Command'),
		('bit', 'Bit'),
	)
	annotation_rows = (
		('cmd', 'Commands', (Ann.START,Ann.CMD,)),
		('bits', 'Bits', (Ann.BITS,)),
	)
	startSequenceBits=0
	lastStartSequenceBitTimestamp=0
	risingClockSample=0
	data=0
	nbits=0
	startWordSample=0

	def log(self,x):
		with open('c:/tmp/p.log','a+') as f:
			f.write(str(x))
			f.write('\n')

	def __init__(self):
		self.reset()
		if os.path.exists("c:/tmp/p.log"):
			os.remove("c:/tmp/p.log")

	def reset(self):
		self.state = State.CMD
		self.startSequenceBits=0
		self.lastStartSequenceBitTimestamp=0
		self.risingClockSample=0
		self.data=0
		self.nbits=0
		self.startWordSample=0
		
	def metadata(self, key, value):
		if key == srd.SRD_CONF_SAMPLERATE:
			self.samplerate = value

	def start(self):
		self.out_ann = self.register(srd.OUTPUT_ANN)

	def decode(self):
		lastPins=(-1,-1,-1)
		while True:
			pins = self.wait([{Pin.RST: 'e'},{Pin.CLK:'e'}])
			if lastPins[Pin.RST]!=pins[Pin.RST]:
				dt=self.samplenum/self.samplerate-self.lastStartSequenceBitTimestamp
				nbits=round(dt/0.010)
				if nbits>0:
					for i in range(nbits):
						self.startSequenceBits<<=1
						if pins[Pin.RST]==0:
							self.startSequenceBits+=1
						self.startSequenceBits&=0xFFFFFF
						if self.startSequenceBits==0x9e1cb6 or self.startSequenceBits==0xae1cb6:
							es=int((self.lastStartSequenceBitTimestamp+(i+1)*0.010)*self.samplerate)
							ss=int(es-24*0.010*self.samplerate)
							if ss<0: ss=0
							self.put(ss,es,self.out_ann,[Ann.START,['START']])
				self.lastStartSequenceBitTimestamp=self.samplenum/self.samplerate
				self.log('nbits '+str(nbits)+' '+hex(self.startSequenceBits)+' '+str(self.lastStartSequenceBitTimestamp))
			if lastPins[Pin.CLK]!=-1 and lastPins[Pin.CLK]!=pins[Pin.CLK]:
				if pins[Pin.CLK]==1:
					self.risingClockSample=self.samplenum
					if self.nbits==0: self.startWordSample=self.samplenum
					self.data<<=1
					self.data+=pins[Pin.DAT]
					self.nbits+=1
					if self.state==State.CMD and self.nbits==24:
						cmd=self.data & 0x3F
						cmdName=''
						if self.data==0x5AA503:
							cmdName='start'
						elif self.data==0XF78F0:
							cmdName='stop'
						elif cmd in cmdValues.keys():
							cmdName='{:s}({:04X})'.format(cmdValues.get(cmd),self.data>>6)
						else:
							cmdName='<unk cmd {:02X}>({:04X})'.format(cmd,self.data>>6)
						self.put(self.startWordSample,self.samplenum,self.out_ann,[Ann.CMD,[cmdName]])
						self.put(self.startWordSample,self.samplenum,self.out_ann,[Ann.BITS,['{:06X}'.format(self.data)]])
						if self.data!=0x5AA503 and self.data!=0XF78F0 and cmd<0x22:
							self.state=State.DATA
						self.nbits=0
						self.data=0
					elif self.state==State.DATA and self.nbits==8:
						self.put(self.startWordSample,self.samplenum,self.out_ann,[Ann.BITS,['{:02X}'.format(self.data)]])
						self.state=State.ACK
						self.nbits=0
						self.data=0
					elif self.state==State.ACK and self.nbits==1:
						if self.data==1: 
							self.state=State.CMD
						else:
							self.state=State.DATA
						self.nbits=0
						self.data=0
						
			lastPins=pins