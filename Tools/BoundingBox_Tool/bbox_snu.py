#!/usr/bin/env python
#-------------------------------------------------------------------------------
#   Object bounding box label tool for YOLO dataset
#
#-------------------------------------------------------------------------------
from __future__ import division
from tkinter import *
# import tkMessageBox
from PIL import Image, ImageTk
import os, glob, random, sys


# colors for the bboxes
#COLORS = ['red', 'blue', 'pink', 'cyan', 'green', 'black']
COLORS=[]
history=[]

max_class_no = 80


class LabelTool():
    def load_classnames(self):
      global max_class_no, class_filename
      f=open(class_filename,"r")
      num_class=0
      while True:
        line = f.readline()
        if not line: break
        if len(line)==1: break
        classname=line.rstrip('\n') #remove return
        # print(len(line), classname,len(self.class_names))
        self.class_names.append(classname)
        num_class=num_class+1
      f.close()
      print(num_class, " classes")
      max_class_no=num_class
      self.num_total_classes=max_class_no

      


      # for i in range(0,max_class_no):
      #   self.class_names.append('Class_'+str(i))

    def __init__(self, master):
        global image_dir
        
        self.random_color()
        
        # set up the main frame
        self.parent = master
        self.parent.title("LabelTool")
        self.frame = Frame(self.parent)
        self.frame2 = Frame(self.parent)
        self.frame3 = Frame(self.parent)
        self.frame4 = Frame(self.parent)
        # self.frame.pack(fill=BOTH, expand=1)

        self.myscrollbar=Scrollbar(self.frame2,orient="vertical")
       

        self.frame.grid(row=0,column=0)
        self.frame2.grid(row=0,column=1)
        self.frame3.grid(row=0,column=2)
        self.frame4.grid(row=1,column=1)


  
        self.parent.resizable(width = FALSE, height = FALSE)


        # initialize global state
        self.imageDir = image_dir
        self.imageList= []
        self.egDir = ''
        self.egList = []
        self.outDir = ''
        self.cur = 0
        self.total = 0
        self.category = 0
        self.selected_class=IntVar()
        self.selected_class.set(0)
        self.imagename = ''
        self.labelfilename = ''
        self.tkimg = None

        # initialize mouse state
        self.STATE = {}
        self.STATE['click'] = 0
        self.STATE['x'], self.STATE['y'] = 0, 0

        # reference to bbox
        self.bboxIdList = []
        # self.bboxTestIdList = []
        self.bboxId = None
        self.bboxList = []
        self.hl = None
        self.vl = None

        # ----------------- GUI stuff ---------------------
        # dir entry & load
        # self.label = Label(self.frame, text = "Image Dir:")
        # self.label.grid(row = 0, column = 0, sticky = E)
        # self.entry = Entry(self.frame)
        # self.entry.grid(row = 0, column = 1, sticky = W+E)
        # self.ldBtn = Button(self.frame, text = "Load", command = self.loadDir)
        # self.ldBtn.grid(row = 0, column = 2, sticky = W+E)

        # main panel for labeling
        self.mainPanel = Canvas(self.frame, cursor='tcross')
        self.mainPanel.bind("<Button-1>", self.mouseClick)
        self.mainPanel.bind("<Motion>", self.mouseMove)
        self.parent.bind("<Escape>", self.cancelBBox)  # press <Espace> to cancel current bbox
        self.parent.bind("<Left>", self.prevImage) # press 'left' to select previous image
        self.parent.bind("<Right>", self.nextImage) # press 'right' to select next image
        self.parent.bind("<Up>", self.prevClass) # press 'up' to select previous class
        self.parent.bind("<Down>", self.nextClass) # press 'down' to select 2 class
        #self.parent.bind("<Down>", self.nextClass) # press 'down' to select 2 class


        self.parent.bind("`", self.selClass0) # press 'd' to go forward
        self.parent.bind("1", self.selClass1) # press 'd' to go forward
        self.parent.bind("2", self.selClass2) # press 'd' to go forward
        self.parent.bind("3", self.selClass3) # press 'd' to go forward
        self.parent.bind("4", self.selClass4) # press 'd' to go forward
        self.parent.bind("5", self.selClass5) # press 'd' to go forward
        self.parent.bind("6", self.selClass6) # press 'd' to go forward
        self.parent.bind("7", self.selClass7) # press 'd' to go forward
        self.parent.bind("8", self.selClass8) # press 'd' to go forward
        self.parent.bind("9", self.selClass9) # press 'd' to go forward
        self.parent.bind("0", self.selClass10) # press 'd' to go forward

        self.parent.bind("a1",self.selClass11)
        self.parent.bind("a2",self.selClass12)
        self.parent.bind("a3",self.selClass13)
        self.parent.bind("a4",self.selClass14)
        self.parent.bind("a5",self.selClass15)
        self.parent.bind("a6",self.selClass16)
        self.parent.bind("a7",self.selClass17)
        self.parent.bind("a8",self.selClass18)
        self.parent.bind("a9",self.selClass19)
        self.parent.bind("a0",self.selClass20)

        self.parent.bind("s1",self.selClass21)
        self.parent.bind("s2",self.selClass22)
        self.parent.bind("s3",self.selClass23)
        self.parent.bind("s4",self.selClass24)
        self.parent.bind("s5",self.selClass25)
        self.parent.bind("s6",self.selClass26)
        self.parent.bind("s7",self.selClass27)
        self.parent.bind("s8",self.selClass28)
        self.parent.bind("s9",self.selClass29)
        self.parent.bind("s0",self.selClass30)

        self.parent.bind("d1",self.selClass31)
        self.parent.bind("d2",self.selClass32)
        self.parent.bind("d3",self.selClass33)
        self.parent.bind("d4",self.selClass34)
        self.parent.bind("d5",self.selClass35)
        self.parent.bind("d6",self.selClass36)
        self.parent.bind("d7",self.selClass37)
        self.parent.bind("d8",self.selClass38)
        self.parent.bind("d9",self.selClass39)
        self.parent.bind("d0",self.selClass40)

        self.parent.bind("f1",self.selClass41)
        self.parent.bind("f2",self.selClass42)
        self.parent.bind("f3",self.selClass43)
        self.parent.bind("f4",self.selClass44)
        self.parent.bind("f5",self.selClass45)
        self.parent.bind("f6",self.selClass46)
        self.parent.bind("f7",self.selClass47)
        self.parent.bind("f8",self.selClass48)
        self.parent.bind("f9",self.selClass49)
        self.parent.bind("f0",self.selClass50)

        #self.parent.bind("<KeyPress>", self.keydown)
        #self.parent.bind("<KeyRelease>", self.keyup)
                
        self.mainPanel.grid(row = 1, column = 1, rowspan = 4, sticky = W+N)


        self.parent.bind("c", self.loadBbox)
        self.parent.bind("z", self.zBbox)


        # showing bbox info & delete bbox
        self.lb1 = Label(self.frame, text = 'Bounding boxes:')
        self.lb1.grid(row = 1, column = 2,  sticky = W+N)
        self.listbox = Listbox(self.frame, width = 22, height = 26)
        self.listbox.grid(row = 2, column = 2, columnspan=2,sticky = N)
        self.btnDel = Button(self.frame, text = 'Delete', command = self.delBBox)
        self.btnDel.grid(row = 3, column = 2, sticky = W+N)
        self.btnClear = Button(self.frame, text = 'Clear All', command = self.clearBBox)
        self.btnClear.grid(row = 3, column = 3, sticky = W+N)

        self.num_total_classes=1
        self.class_names=[]
        self.classbuttons=[]

        self.load_classnames()

        for i in range(0,max_class_no):
          print(i,self.class_names[i])
          if (max_class_no>30) and (i >=max_class_no/2):
            self.classbuttons.append( Radiobutton(self.frame3, text = str(i)+'. '+self.class_names[i], variable=self.selected_class, value=i) )
          else:
            self.classbuttons.append( Radiobutton(self.frame2, text = str(i)+'. '+self.class_names[i], variable=self.selected_class, value=i) )
          self.classbuttons[i].grid(row = i, column = 4, sticky = W+N)
          


        # self.classEntry = Entry(self.frame, width = 10)
        # self.classEntry = Entry(self.frame4, width = 10)
        # self.classEntry.grid(row = max_class_no+2, column = 4, sticky = W+E+N)
        # self.classSetBtn = Button(self.frame, text = 'Class Name Set', command = self.classSet)


        # self.classSetBtn = Button(self.frame4, text = 'Class Name Set', command = self.classSet)
        # self.classSetBtn.grid(row = max_class_no+3, column = 4, sticky = W+E+N)


        # control panel for image navigation
        self.ctrPanel = Frame(self.frame)
        self.ctrPanel.grid(row = 21, column = 1, columnspan = 2, sticky = W+E)
        self.prevBtn = Button(self.ctrPanel, text='<< Prev', width = 10, command = self.prevImage)
        self.prevBtn.pack(side = LEFT, padx = 5, pady = 3)
        self.nextBtn = Button(self.ctrPanel, text='Next >>', width = 10, command = self.nextImage)
        self.nextBtn.pack(side = LEFT, padx = 5, pady = 3)
        self.progLabel = Label(self.ctrPanel, text = "Progress:     /    ")
        self.progLabel.pack(side = LEFT, padx = 5)
        self.tmpLabel = Label(self.ctrPanel, text = "Go to Image No.")
        self.tmpLabel.pack(side = LEFT, padx = 5)
        self.idxEntry = Entry(self.ctrPanel, width = 5)
        self.idxEntry.pack(side = LEFT)
        self.goBtn = Button(self.ctrPanel, text = 'Go', command = self.gotoImage)
        self.goBtn.pack(side = LEFT)

        # display mouse position
        self.disp = Label(self.ctrPanel, text='')
        self.disp.pack(side = RIGHT)

        self.frame.columnconfigure(1, weight = 1)
        self.frame.rowconfigure(4, weight = 1)

        self.loadDir()

    def random_color(self):
        for i in range(max_class_no) : 
            color = ["#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)])]
            COLORS.append(color)
        


    def loadDir(self, dbg = False):
        global image_dir
        self.category = 1
        # self.imageDir = r'../../../Logs/Images/current'
        self.imageDir=image_dir
        self.imageList = sorted(glob.glob(os.path.join(self.imageDir, '*.png')))
        if len(self.imageList) == 0:
            print ('No .png images found in the specified dir!')
            return

        # default to the 1st image in the collection
        self.cur = 1
        self.total = len(self.imageList)
        self.outDir = image_dir+r'/Labeled'
        if not os.path.exists(self.outDir): os.mkdir(self.outDir)
        self.loadImage()
        print ('%d images loaded' %(self.total))

    def loadImage(self):
        # load image
        imagepath = self.imageList[self.cur - 1]
        self.img = Image.open(imagepath)
        self.tkimg = ImageTk.PhotoImage(self.img)
        self.mainPanel.config(width = max(self.tkimg.width(), 400), height = max(self.tkimg.height(), 400))
        self.mainPanel.create_image(0, 0, image = self.tkimg, anchor=NW)
        self.progLabel.config(text = "%s  %04d/%04d" %(imagepath, self.cur, self.total))

        # load labels
        self.clearBBox()
        self.imagename = os.path.split(imagepath)[-1].split('.')[0]
        labelname = self.imagename + '.txt'
        self.labelfilename = os.path.join(self.outDir, labelname)
        self.objdatafilename=os.path.join(self.outDir, "obj.data")
        self.objnamesfilename=os.path.join(self.outDir, "obj.names")

        bbox_cnt = 0
        if os.path.exists(self.labelfilename):
            with open(self.labelfilename) as f:
                for (i, line) in enumerate(f):
                    tmp = [float(t.strip()) for t in line.split()]
                    print (tmp)

                    #xcenter ycenter xwidth ywidth!!!
                    w=self.img.size[0]
                    h=self.img.size[1]
                    s_class=int(tmp[0])
                    x0 = ( tmp[1]-tmp[3]*0.5 )*w
                    x1 = ( tmp[1]+tmp[3]*0.5)*w
                    y0 = ( tmp[2]-tmp[4]*0.5)*h
                    y1 = ( tmp[2]+tmp[4]*0.5)*h

                    self.bboxList.append((s_class,x0,y0,x1,y1))
                    tmpId = self.mainPanel.create_rectangle(x0,y0,x1,y1,\
                                                            width = 2, \
                                                            outline = COLORS[s_class % len(COLORS)])
                    tmp_text_Id = self.mainPanel.create_text(x0 +9, y0 -7, \
                                                            text=self.class_names[int(tmp[0])], \
                                                            fill=COLORS[s_class % len(COLORS)], \
                                                            font=('Helvetica 15 bold'),
                                                            ) # outline=COLORS[s_class % len(COLORS)]
                    self.bboxIdList.append([tmpId,tmp_text_Id])
                    self.listbox.insert(END, '%d: (%d, %d) -> (%d, %d)' %(s_class,x0,y0,x1,y1))
                    self.listbox.itemconfig(len(self.bboxIdList) - 1, fg = COLORS[s_class % len(COLORS)])

        #SJ: disable reading object names from obj.names
        #We now use classnames.txt

        # if os.path.exists(self.objnamesfilename):
        #     with open(self.objnamesfilename) as f:
        #         self.num_total_classes=0
        #         for (i, line) in enumerate(f):
        #             tmp = [t.strip() for t in line.split()]
        #             self.class_names[i]=tmp[0]
        #             self.num_total_classes+=1
        #             self.classbuttons[i].config(text=tmp[0])

    def loadBbox(self,event):
        print("loadBbox")
        imagepath = self.imageList[self.cur - 1]
        
        bbx_imagepath = self.imageList[self.cur - 2]
        self.img = Image.open(imagepath)
        self.tkimg = ImageTk.PhotoImage(self.img)
        self.mainPanel.config(width = max(self.tkimg.width(), 400), height = max(self.tkimg.height(), 400))
        self.mainPanel.create_image(0, 0, image = self.tkimg, anchor=NW)
        self.progLabel.config(text = "%s  %04d/%04d" %(imagepath, self.cur, self.total))

        # load labels
        self.clearBBox()
        self.bbx_imagename = os.path.split(bbx_imagepath)[-1].split('.')[0]
        bbx_labelname = self.bbx_imagename + '.txt'
        self.bbx_labelfilename = os.path.join(self.outDir, bbx_labelname)

        self.imagename = os.path.split(imagepath)[-1].split('.')[0]
        labelname = self.imagename + '.txt'
        self.labelfilename = os.path.join(self.outDir, labelname)


        self.objdatafilename=os.path.join(self.outDir, "obj.data")
        self.objnamesfilename=os.path.join(self.outDir, "obj.names")

        bbox_cnt = 0
        if os.path.exists(self.bbx_labelfilename):
            with open(self.bbx_labelfilename) as f:
                for (i, line) in enumerate(f):
                    tmp = [float(t.strip()) for t in line.split()]
                    print (tmp)

                    #xcenter ycenter xwidth ywidth!!!
                    w=self.img.size[0]
                    h=self.img.size[1]
                    s_class=int(tmp[0])
                    x0 = ( tmp[1]-tmp[3]*0.5 )*w
                    x1 = ( tmp[1]+tmp[3]*0.5)*w
                    y0 = ( tmp[2]-tmp[4]*0.5)*h
                    y1 = ( tmp[2]+tmp[4]*0.5)*h

                    self.bboxList.append((s_class,x0,y0,x1,y1))
                    tmpId = self.mainPanel.create_rectangle(x0,y0,x1,y1,\
                                                            width = 2, \
                                                            outline = COLORS[s_class % len(COLORS)])
                    tmp_text_Id = self.mainPanel.create_text(x0 + 9, y0 - 7, \
                                                             text=self.class_names[int(tmp[0])], \
                                                             fill=COLORS[s_class % len(COLORS)], \
                                                             font=('Helvetica 15 bold'))
                    self.bboxIdList.append([tmpId,tmp_text_Id])
                    self.listbox.insert(END, '%d: (%d, %d) -> (%d, %d)' %(s_class,x0,y0,x1,y1))
                    self.listbox.itemconfig(len(self.bboxIdList) - 1, fg = COLORS[s_class % len(COLORS)])
        
        if os.path.exists(self.objnamesfilename):
            with open(self.objnamesfilename) as f:
                self.num_total_classes=0
                for (i, line) in enumerate(f):
                    tmp = [t.strip() for t in line.split()]
                    self.class_names[i]=tmp[0]
                    self.num_total_classes+=1

    def saveImage(self):
      # We should save EVERY image (for negative example)
      # if len(self.bboxList)>0:

      with open(self.labelfilename, 'w') as f:
        # f.write('%d' %len(self.bboxList))
        for bbox in self.bboxList:
          print (bbox[0], bbox[1], bbox[2], bbox[3],bbox[4])
          #Class x0 y0 x1 y1
          xcenter=(bbox[1]+bbox[3])/2
          ycenter=(bbox[2]+bbox[4])/2
          xwidth=(bbox[3]-bbox[1])
          ywidth=(bbox[4]-bbox[2])
          w=self.img.size[0]
          h=self.img.size[1]
          f.write('%d ' %bbox[0])
          f.write('%f ' %(xcenter/w))
          f.write('%f ' %(ycenter/h))
          f.write('%f ' %(xwidth/w))
          f.write('%f\n' %(ywidth/h))
        str='cp '+self.imageList[self.cur - 1] + ' '+self.outDir
        print(str)
        os.system(str)
        print ('Image No. %d saved' %(self.cur))
        with open(self.objdatafilename, 'w') as f:
          f.write('classes=%d\ntrain = train.txt\nvalid=test.txt\nnames=obj.names\nbackup=backup/' %self.num_total_classes)
        with open(self.objnamesfilename, 'w') as f:
          for i in range(0,self.num_total_classes): f.write('%s\n' %self.class_names[i])

    def mouseClick(self, event):
        if self.STATE['click'] == 0:
            self.STATE['x'], self.STATE['y'] = event.x, event.y
        else:
            x1, x2 = min(self.STATE['x'], event.x), max(self.STATE['x'], event.x)
            y1, y2 = min(self.STATE['y'], event.y), max(self.STATE['y'], event.y)
            self.bboxList.append((self.selected_class.get(), x1, y1, x2, y2))
            self.bboxIdList.append([self.bboxId,self.bboxId+1])
            self.bboxId = None
            self.listbox.insert(END, '%d: (%d, %d) -> (%d, %d)' %(self.selected_class.get(), x1, y1, x2, y2))
            self.listbox.itemconfig(len(self.bboxIdList) - 1, fg = COLORS[self.selected_class.get() % len(COLORS)])
        self.STATE['click'] = 1 - self.STATE['click']

    def mouseMove(self, event):
        self.disp.config(text = 'x: %d, y: %d' %(event.x, event.y))
        if self.tkimg:
            if self.hl:
                self.mainPanel.delete(self.hl)
            self.hl = self.mainPanel.create_line(0, event.y, self.tkimg.width(), event.y, width = 2)
            if self.vl:
                self.mainPanel.delete(self.vl)
            self.vl = self.mainPanel.create_line(event.x, 0, event.x, self.tkimg.height(), width = 2)
        if 1 == self.STATE['click']:
            if self.bboxId:
                self.mainPanel.delete(self.bboxId)
                self.mainPanel.delete(self.tmp_text_Id)
            self.bboxId = self.mainPanel.create_rectangle(self.STATE['x'], self.STATE['y'], \
                                                            event.x, event.y, \
                                                            width = 2, \
                                                            outline = COLORS[self.selected_class.get()])
            self.tmp_text_Id = self.mainPanel.create_text(self.STATE['x'] + 9, self.STATE['y'] - 7, \
                                                     text=self.class_names[int(self.selected_class.get())], \
                                                     fill=COLORS[self.selected_class.get()], \
                                                     font=('Helvetica 15 bold'))

    def cancelBBox(self, event):
        if 1 == self.STATE['click']:
            if self.bboxId:
                self.mainPanel.delete(self.bboxId)
                self.bboxId = None
                self.STATE['click'] = 0

    def delBBox(self):
        sel = self.listbox.curselection()
        if len(sel) != 1 :
            return
        idx = int(sel[0])
        self.mainPanel.delete(self.bboxIdList[idx][1])
        self.mainPanel.delete(self.bboxIdList[idx][0])
        self.bboxIdList.pop(idx)
        self.bboxList.pop(idx)
        self.listbox.delete(idx)

    def zBbox(self,event):
        idx = len(self.bboxList)-1
        self.mainPanel.delete(self.bboxIdList[idx][1])
        self.mainPanel.delete(self.bboxIdList[idx][0])
        self.bboxIdList.pop(idx)
        self.bboxList.pop(idx)
        self.listbox.delete(idx)

    def clearBBox(self):
        print('self.bboxIdList', self.bboxIdList)
        for idx in range(len(self.bboxIdList)):
            self.mainPanel.delete(self.bboxIdList[idx][1])
            self.mainPanel.delete(self.bboxIdList[idx][0])
        self.listbox.delete(0, len(self.bboxList))
        self.bboxTestIdList = []
        self.bboxIdList = []
        self.bboxList = []

    def selClass0(self,event=None):
        self.selected_class.set(0)
    def selClass1(self,event=None):
        self.selected_class.set(1)
    def selClass2(self,event=None):
        self.selected_class.set(2)
    def selClass3(self,event=None):
        self.selected_class.set(3)
    def selClass4(self,event=None):
        self.selected_class.set(4)
    def selClass5(self,event=None):
        self.selected_class.set(5)
    def selClass6(self,event=None):
        self.selected_class.set(6)
    def selClass7(self,event=None):
        self.selected_class.set(7)
    def selClass8(self,event=None):
        self.selected_class.set(8)
    def selClass9(self,event=None):
        self.selected_class.set(9)
    def selClass10(self,event=None):
        self.selected_class.set(10)
    def selClass11(self,event=None):
        self.selected_class.set(11)
    def selClass12(self,event=None):
        self.selected_class.set(12)
    def selClass13(self,event=None):
        self.selected_class.set(13)
    def selClass14(self,event=None):
        self.selected_class.set(14)
    def selClass15(self,event=None):
        self.selected_class.set(15)
    def selClass16(self,event=None):
        self.selected_class.set(16)
    def selClass17(self,event=None):
        self.selected_class.set(17)
    def selClass18(self,event=None):
        self.selected_class.set(18)
    def selClass19(self,event=None):
        self.selected_class.set(19)
    def selClass20(self,event=None):
        self.selected_class.set(20)
    def selClass21(self,event=None):
        self.selected_class.set(21)
    def selClass22(self,event=None):
        self.selected_class.set(22)
    def selClass23(self,event=None):
        self.selected_class.set(23)
    def selClass24(self,event=None):
        self.selected_class.set(24)
    def selClass25(self,event=None):
        self.selected_class.set(25)
    def selClass26(self,event=None):
        self.selected_class.set(26)
    def selClass27(self,event=None):
        self.selected_class.set(27)
    def selClass28(self,event=None):
        self.selected_class.set(28)
    def selClass29(self,event=None):
        self.selected_class.set(29)
    def selClass30(self,event=None):
        self.selected_class.set(30)
    def selClass31(self,event=None):
        self.selected_class.set(31)
    def selClass32(self,event=None):
        self.selected_class.set(32)
    def selClass33(self,event=None):
        self.selected_class.set(33)
    def selClass34(self,event=None):
        self.selected_class.set(34)
    def selClass35(self,event=None):
        self.selected_class.set(35)
    def selClass36(self,event=None):
        self.selected_class.set(36)
    def selClass37(self,event=None):
        self.selected_class.set(37)
    def selClass38(self,event=None):
        self.selected_class.set(38)
    def selClass39(self,event=None):
        self.selected_class.set(39)
    def selClass40(self,event=None):
        self.selected_class.set(40)
    def selClass41(self,event=None):
        self.selected_class.set(41)
    def selClass42(self,event=None):
        self.selected_class.set(42)
    def selClass43(self,event=None):
        self.selected_class.set(43)
    def selClass44(self,event=None):
        self.selected_class.set(44)
    def selClass45(self,event=None):
        self.selected_class.set(45)
    def selClass46(self,event=None):
        self.selected_class.set(46)
    def selClass47(self,event=None):
        self.selected_class.set(47)
    def selClass48(self,event=None):
        self.selected_class.set(48)
    def selClass49(self,event=None):
        self.selected_class.set(49)
    def selClass50(self,event=None):
        self.selected_class.set(50)

    
        
    def mulClass(self,event):
        history=[]
        history.append(event.keycode)
        print(event.keycode)
        print(history)
        var = str(history)
        self.selected_class.set(var)
    
    '''
    def keyup(self,e):
        print e.keycode
        if  e.keycode in history :
            history.pop(history.index(e.keycode))
        var = str(history)
        print("dddddd")
        print(var)

    def keydown(self,e):
        history=[]
        if not e.keycode in history :
            history.append(e.keycode)
        print(e.keycode)
        print(history)
        var = str(history)
        print("ggggg")
        print(var)
    '''
    



    def prevClass(self, event = None):
        cclass= self.selected_class.get()
        cclass-=1
        if cclass<0: cclass=0
        self.selected_class.set(cclass)

    def nextClass(self, event = None):
        cclass= self.selected_class.get()
        cclass+=1
        if cclass>max_class_no-1: cclass=max_class_no-1
        self.selected_class.set(cclass)

    def prevImage(self, event = None):
        self.saveImage()
        if self.cur > 1:
            self.cur -= 1
            self.loadImage()

    def nextImage(self, event = None):
        self.saveImage()
        if self.cur < self.total:
            self.cur += 1
            self.loadImage()

    def gotoImage(self):
        idx = int(self.idxEntry.get())
        if 1 <= idx and idx <= self.total:
            self.saveImage()
            self.cur = idx
            self.loadImage()

    # def classSet(self):
    #     cname = self.classEntry.get()
    #     cclass= self.selected_class.get()
    #     print(cname)
    #     self.classbuttons[cclass].config(text=cname)
    #     self.class_names[cclass]=cname
    #     if (self.num_total_classes<cclass+1):self.num_total_classes=cclass+1
    #     self.classEntry.delete(0,len(cname))

# todo: reset the class name box


if __name__ == '__main__':
  global image_dir
  if len(sys.argv)>1:
    image_dir=sys.argv[1]
    class_filename=sys.argv[1]+"classnames.txt"
  else:
    print("using default path")
    image_dir='./Images/Test'
    class_filename='./Images/Test/classnames.txt'

  root = Tk()
  tool = LabelTool(root)

  root.resizable(width =  True, height = True)
  root.mainloop()
