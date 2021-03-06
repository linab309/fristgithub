package smart.ch.gehangzigbee;

import android.os.Bundle;
import android.app.Activity;
import android.view.Menu;
import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.ContentProviderOperation.Builder;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.res.Resources;
import android.graphics.drawable.Drawable;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.widget.AbsoluteLayout;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.RadioButton;
import android.widget.ImageView;
import android.widget.ToggleButton;

public class ZigbeeActivity extends Activity {

	private static final String TAG = "ZigbeeActivity";
	private String SOCKET_IP = "192.168.1.250";
	//private String SOCKET_IP = "192.168.50.247";
	private int SOCKET_PORT = 13501;
	Socket socket = null;
	byte mBufferReceived[] = new byte[1024];
	byte mBufferSend[] = new byte[1024];
	String mStrSend = "12346";
	Object mLock = new Object();
	private static final boolean mIsTest = true;
	
	private static final int PAGE_STATUS = 1;
	private static final int PAGE_LIST = 2;
	private static final int PAGE_CONTROL = 3;
	private static final int PAGE_SET = 4;
	private int mPage = PAGE_STATUS;

	private static final int STATUS_LINK = 1;
	private static final int STATUS_GET_LIST = 2;
	private static final int STATUS_FAIL_LINK = 3;
	private static final int STATUS_FAIL_GET_LIST = 4;

	private static final int PROTOCOL_CMDID_START = 2;
	private static final int PROTOCOL_DATA_START = 4;
	private int mStatus = STATUS_LINK;
	private ViewGroup mPageStatus;
	private ViewGroup mPageList;
	private ViewGroup mPageControl;
	private ViewGroup mPageSet;

	private TextView mTextStatus;
	private Button mBtnStatus;
	Button mBtnConnect = null;
	Button mBtnDisconnect = null;
	Button mBtnSend = null;
	private ViewGroup mlayoutList;
	private static final int DEVICE_MAX_NUM = 256;
	private static final int GROUP_MAX_NUM = 256;
	private ArrayList<DeviceInfo> mDeviceInfoList = new ArrayList<DeviceInfo>(DEVICE_MAX_NUM);
	private ArrayList<GroupInfo> mGroupInfoList = new ArrayList<GroupInfo>(GROUP_MAX_NUM);
	private ArrayList<ItemInfo> mItemInfoList = new ArrayList<ItemInfo>();

	private ReceiveThread mReceiveThread;
	private final int MSG_LINK = 1;
	private final int MSG_GET_LIST_OK = 2;
	Handler mHandler = new Handler()
	{
		public void handleMessage(Message msg)
		{
			switch(msg.what)
			{
			case MSG_LINK:
				if(socket != null)
				{
					setStatus(STATUS_GET_LIST);
					
					new Thread(){
						public void run(){
				           	try { 
			           		 	if(socket != null)
			           		 	{
							    	OutputStream out = socket.getOutputStream();
									int startIndex = 0;
									int maxReadLen = 256 - 2 - 4;

									////in: 	startIndex[2],MaxReadLength[2]
									////out:	startIndex[2],ReadedLength[2],item_len[i],item_info[i]
									////		if item_len[i] == 1,means null item,only crc8
									mBufferSend[PROTOCOL_DATA_START+0] = (byte)startIndex;
									mBufferSend[PROTOCOL_DATA_START+1] = (byte)(startIndex>>8);
									mBufferSend[PROTOCOL_DATA_START+2] = (byte)maxReadLen;
									mBufferSend[PROTOCOL_DATA_START+3] = (byte)(maxReadLen>>8);
									int len = fillSendBufferCmd(mBufferSend,0x0002,4);
									
								    out.write(mBufferSend,0,len);
								    out.flush();
			           		 	}
			            	    
			            	} catch(Exception e) { 
			            	    Log.e("TCP", "S: Error", e); 
								setStatus(STATUS_FAIL_GET_LIST);
			            	} finally { 
			            	    
			            	} 
						}
					}.start();
				}
				else
					setStatus(STATUS_FAIL_LINK);
				break;
			case MSG_GET_LIST_OK:
				{
					// make mItemInfoList
					//mItemInfoList.clear();
					int i;
					for(i=0;i<mDeviceInfoList.size();i++)
					{
						DeviceInfo deviceInfo = mDeviceInfoList.get(i);
						if( ! deviceInfo.isUsed)
							continue;
							
						Log.d(TAG,String.format("device[%d]=%b,%x,%x",i,deviceInfo.isUsed,deviceInfo.addr,deviceInfo.groupId));
						/*
						if(deviceInfo.groupId == 0)
						{
							ItemInfo itemInfo = new ItemInfo();
							itemInfo.type = ItemInfo.TYPE_LIGHT;
							itemInfo.id = deviceInfo.id;
							itemInfo.name = deviceInfo.deviceName;
							mItemInfoList.add(itemInfo);
						}
						else
						{
							boolean isAlreadyAdded = false;
							int k;
							for(k=0;k<mItemInfoList.size();k++)
							{
								ItemInfo itemInfo = mItemInfoList.get(k);
								Log.d(TAG,String.format("find itemInfo[%d]=%x,%x,%s",k,itemInfo.type,itemInfo.id,itemInfo.name));
								if(itemInfo.type == ItemInfo.TYPE_GROUP && itemInfo.id == deviceInfo.groupId)
								{
									isAlreadyAdded = true;
									break;
								}
							}
							if( ! isAlreadyAdded)
							{
								ItemInfo itemInfo = new ItemInfo();
								itemInfo.type = ItemInfo.TYPE_GROUP;
								itemInfo.id = deviceInfo.groupId;
								for(k=0;k<mGroupInfoList.size();k++)
								{
									GroupInfo groupInfo = mGroupInfoList.get(k);
									if(groupInfo.groupId == itemInfo.id)
									{
										itemInfo.name = groupInfo.groupName;
										Log.d(TAG,String.format("add group[%d]%s",groupInfo.groupId,itemInfo.name));
										break;
									}
								}
								mItemInfoList.add(itemInfo);
							}
							
						}*/
					}
				}
				break;
			}
		}
	};
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_zigbee);

		mReceiveThread = new ReceiveThread();
		
		/*
		if(mIsTest)
		{
			DeviceInfo deviceInfo;

			deviceInfo = new DeviceInfo();
			deviceInfo.id = 0x2345;
			deviceInfo.groupId = 0;
			deviceInfo.deviceName = String.format("xxxxx");
			mDeviceInfoList.add(deviceInfo);

			deviceInfo = new DeviceInfo();
			deviceInfo.id = 0x2478;
			deviceInfo.groupId = 0;
			deviceInfo.deviceName = String.format("");
			mDeviceInfoList.add(deviceInfo);

			deviceInfo = new DeviceInfo();
			deviceInfo.id = 0x3138;
			deviceInfo.groupId = 1;
			deviceInfo.deviceName = String.format("");
			mDeviceInfoList.add(deviceInfo);

			deviceInfo = new DeviceInfo();
			deviceInfo.id = 0x1818;
			deviceInfo.groupId = 1;
			deviceInfo.deviceName = String.format("");
			mDeviceInfoList.add(deviceInfo);

			deviceInfo = new DeviceInfo();
			deviceInfo.id = 0x3456;
			deviceInfo.groupId = 2;
			deviceInfo.deviceName = String.format("");
			mDeviceInfoList.add(deviceInfo);

			deviceInfo = new DeviceInfo();
			deviceInfo.id = 0x4455;
			deviceInfo.groupId = 2;
			deviceInfo.deviceName = String.format("");
			mDeviceInfoList.add(deviceInfo);

			deviceInfo = new DeviceInfo();
			deviceInfo.id = 0x3322;
			deviceInfo.groupId = 2;
			deviceInfo.deviceName = String.format("");
			mDeviceInfoList.add(deviceInfo);

			GroupInfo groupInfo;
			groupInfo = new GroupInfo();
			groupInfo.groupId = 1;
			groupInfo.groupName = String.format("大厅");
			mGroupInfoList.add(groupInfo);

			groupInfo = new GroupInfo();
			groupInfo.groupId = 2;
			groupInfo.groupName = String.format("卧室");
			mGroupInfoList.add(groupInfo);

		}
		*/
		{
			int i;
			for(i=0;i<mDeviceInfoList.size();i++)
			{
				mDeviceInfoList.get(i).name = new byte[16];
				mDeviceInfoList.get(i).reserved = new byte[4];
			}
			for(i=0;i<mGroupInfoList.size();i++)
				mGroupInfoList.get(i).groupName = new byte[16];
		}
		mPageStatus = (ViewGroup)findViewById(R.id.page_status);
		mPageList = (ViewGroup)findViewById(R.id.page_list);
		mPageControl = (ViewGroup)findViewById(R.id.page_control);
		//mPageSet = (ViewGroup)findViewById(R.id.page_set);

		mlayoutList = (ViewGroup)findViewById(R.id.layout_list);
		{
			int i;
			LayoutInflater mInflater;
			mInflater = (LayoutInflater)getSystemService(LAYOUT_INFLATER_SERVICE);
			for(i=0;i<mItemInfoList.size();	i++)
			{
				ItemInfo itemInfo = mItemInfoList.get(i);
				ViewGroup layout = (ViewGroup)mInflater.inflate(R.layout.list_item,mlayoutList,false);
				Button mBtnItem = (Button)layout.findViewById(R.id.btn_item);
				if(itemInfo.name == null || itemInfo.name.equals(new String("")))
					mBtnItem.setText(String.format("%x",itemInfo.id));
				else
					mBtnItem.setText(itemInfo.name);
				//设置图片
				Drawable drawable = null;
				if(itemInfo.type == itemInfo.TYPE_LIGHT)
					drawable = getResources().getDrawable(R.drawable.ic_light);
				else if(itemInfo.type == itemInfo.TYPE_GROUP)
					drawable = getResources().getDrawable(R.drawable.ic_group);
				drawable.setBounds(0,0,drawable.getMinimumWidth(),drawable.getMinimumHeight());
				mBtnItem.setCompoundDrawables(drawable,null,null,null);

				mBtnItem.setOnClickListener(new View.OnClickListener(){
					public void onClick(View v)
					{
						setPage(PAGE_CONTROL);
					}
				});
				
				mlayoutList.addView(layout);
			}
		}
		mTextStatus = (TextView)findViewById(R.id.text_status);
		mBtnStatus = (Button)findViewById(R.id.btn_status);
		mBtnStatus.setOnClickListener(new View.OnClickListener(){
			public void onClick(View v)
			{
				finish();
			}
		});
		new Thread(){
			public void run(){
	           	try { 
           		 	String inputIpAddr=SOCKET_IP;               		
           		 
           		 	InetAddress serverAddr = InetAddress.getByName(inputIpAddr);//TCPServer.SERVERIP 
                	Log.d("TCP", "C: Connecting..."); 
                	//socket = new Socket(serverAddr, SOCKET_PORT);  
                	socket = new Socket(inputIpAddr, SOCKET_PORT);

            	    
            	} catch(Exception e) { 
            	    Log.e("TCP", "S: Error", e); 
					socket = null;
            	} finally { 
            	    mHandler.sendEmptyMessage(MSG_LINK);
            	} 
			}
		}.start();
		
		/*
		mBtnConnect = (Button)findViewById(R.id.btn_connect);
		mBtnConnect.setOnClickListener(new View.OnClickListener(){
			public void onClick(View v)
			{
				new Thread(){
					public void run(){
			           	try { 
		           		 	String inputIpAddr=SOCKET_IP;               		
		           		 
		           		 	InetAddress serverAddr = InetAddress.getByName(inputIpAddr);//TCPServer.SERVERIP 
		                	Log.d("TCP", "C: Connecting..."); 
		                	//socket = new Socket(serverAddr, SOCKET_PORT);  
		                	socket = new Socket(inputIpAddr, SOCKET_PORT);

		            	    
		            	} catch(Exception e) { 
		            	    Log.e("TCP", "S: Error", e); 
		            	} finally { 
		            	    
		            	} 
					}
				}.start();
			}
		});
		mBtnDisconnect = (Button)findViewById(R.id.btn_disconnect);
		mBtnDisconnect.setOnClickListener(new View.OnClickListener(){
			public void onClick(View v)
			{
	           	 try { 
	           		 	if(socket != null)
	           		 	{
							socket.close();
							socket = null;
	           		 	}
	            	    
	            	} catch(Exception e) { 
	            	    Log.e("TCP", "S: Error", e); 
	            	} finally { 
	            	    
	            	} 
			}
		});
		mBtnSend = (Button)findViewById(R.id.btn_send);
		mBtnSend.setOnClickListener(new View.OnClickListener(){
			public void onClick(View v)
			{
				new Thread(){
					public void run(){
			           	try { 
		           		 	if(socket != null)
		           		 	{
						    	PrintWriter out = new PrintWriter( new BufferedWriter( new OutputStreamWriter(socket.getOutputStream())),true); 
							    out.println(mStrSend);
							    out.flush();
		           		 	}
		            	    
		            	} catch(Exception e) { 
		            	    Log.e("TCP", "S: Error", e); 
		            	} finally { 
		            	    
		            	} 
					}
				}.start();
			}
		});
		*/
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		//getMenuInflater().inflate(R.menu.activity_zigbee, menu);
		return true;
	}
    protected void onDestroy()
	{
		super.onDestroy();
		try { 
			   if(socket != null)
			   {
				   socket.close();
				   socket = null;
			   }
			   
		   } catch(Exception e) { 
			   Log.e("TCP", "S: Error", e); 
		   } finally { 
			   
		   } 
		mReceiveThread.stopThread();
    }
    @Override
    public void onBackPressed() {
		switch(mPage)
		{
		case PAGE_STATUS:
			super.onBackPressed();
			break;
		case PAGE_LIST:
			super.onBackPressed();
			break;
		case PAGE_CONTROL:
			setPage(PAGE_LIST);
			break;
		case PAGE_SET:
			setPage(PAGE_LIST);
			break;
		}
        
    }
	private void setPage(int nPage)
	{
		if(mPage != nPage)
		{
			switch(mPage)
			{
			case PAGE_STATUS:
				mPageStatus.setVisibility(View.GONE);
				break;
			case PAGE_LIST:
				mPageList.setVisibility(View.GONE);
				break;
			case PAGE_CONTROL:
				mPageControl.setVisibility(View.GONE);
				break;
			case PAGE_SET:
				mPageSet.setVisibility(View.GONE);
				break;
			}
			switch(nPage)
			{
			case PAGE_STATUS:
				mPageStatus.setVisibility(View.VISIBLE);
				break;
			case PAGE_LIST:
				mPageList.setVisibility(View.VISIBLE);
				break;
			case PAGE_CONTROL:
				mPageControl.setVisibility(View.VISIBLE);
				break;
			case PAGE_SET:
				mPageSet.setVisibility(View.VISIBLE);
				break;
			}
			mPage = nPage;
		}

	}
	private void setStatus(int status)
	{


		if(mStatus != status)
		{
			switch(status)
			{
			case STATUS_LINK:
				mTextStatus.setText("连接中...");
				mBtnStatus.setText("取消");
				mBtnStatus.setVisibility(View.VISIBLE);
				break;
			case STATUS_GET_LIST:
				mTextStatus.setText("获取设备列表中...");
				mBtnStatus.setVisibility(View.INVISIBLE);
				break;
			case STATUS_FAIL_LINK:
				mTextStatus.setText("连接失败");
				mBtnStatus.setText("返回");
				mBtnStatus.setVisibility(View.VISIBLE);
				break;
			case STATUS_FAIL_GET_LIST:
				mTextStatus.setText("获取列表失败");
				mBtnStatus.setText("返回");
				mBtnStatus.setVisibility(View.VISIBLE);
				break;
			}
			mStatus = status;
		}
	}
	byte CalcChecksum(byte[] buf,int offset,int len)
	{
		int checksum = 0;
		int i;
		for(i=0;i<len;i++)
			checksum += buf[i+offset];
		return (byte)(checksum & 0xff);
	}
	private int fillSendBuffer(byte[] buf,int dataLen)
	{
		buf[0] = (byte)0xAA;
		buf[1] = (byte)dataLen;
		buf[2+dataLen] = CalcChecksum(buf,2,dataLen);
		buf[3+dataLen] = (byte)0x55;
		return dataLen + 4;
	}
	private int fillSendBufferCmd(byte[] buf,int cmdID,int dataLen)
	{
		buf[2] = (byte)cmdID;
		buf[3] = (byte)(cmdID>>8);
		return fillSendBuffer(buf,dataLen+2);
	}
	private class ReceiveThread implements Runnable
	{
		public Thread mThread;
		boolean mIsRun = true;
		int mDealyTime = 60;
		ReceiveThread()
		{
			mThread = new Thread(this);
			mThread.start();
		}
		public void stopThread()
		{
			mIsRun = false;
			//mThread.interrupt();
			try {
				mThread.join();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		public void run() {
			int nIoctrl;
			int nRet;
			long nTime;


			
			while(mIsRun)
			{

				if(socket != null)
				{
					nTime = System.currentTimeMillis();
					if( ! socket.isClosed())
					{
						try{
							InputStream is = socket.getInputStream(); 

							int i;
							//for(i=0;i<mBufferReceived.length;i++)
							//	mBufferReceived[i] = 0;
							int n = is.read(mBufferReceived);
							if(n > 0){						
								Log.d(TAG,String.format("received=") + new String(mBufferReceived));
								int cmdID = (mBufferReceived[PROTOCOL_CMDID_START+0]&0xff)|((mBufferReceived[PROTOCOL_CMDID_START+1]&0xff)<<8);
								int dataLen = (mBufferReceived[1]&0xff);
								Log.d(TAG,String.format("cmdID=%x,len=%d",cmdID,dataLen));
								switch(cmdID)
								{
								case 0x8002://read device list
									{
										////in: 	startIndex[2],MaxReadLength[2]
										////out:	startIndex[2],ReadedLength[2],item_len[i],item_info[i]
										////		if item_len[i] == 1,means null item,only crc8
										int startIndex = (mBufferReceived[PROTOCOL_DATA_START+0]&0xff)|((mBufferReceived[PROTOCOL_DATA_START+1]&0xff)<<8);
										int ReadedLength = (mBufferReceived[PROTOCOL_DATA_START+2]&0xff)|((mBufferReceived[PROTOCOL_DATA_START+3]&0xff)<<8);
										int iProcess = 0;
										int itemReaded = 0;
										Log.d(TAG,String.format("startIndex=%d,ReadedLength=%d",startIndex,ReadedLength));
										while(iProcess < ReadedLength)
										{
											int itemLen = (mBufferReceived[PROTOCOL_DATA_START+4+iProcess]&0xff);
											Log.d(TAG,String.format("itemLen[%d]=%d",PROTOCOL_DATA_START+4+iProcess,itemLen));
//											Log.d(TAG,String.format("itemLen[%d]=%d",PROTOCOL_DATA_START+4+iProcess,itemLen));
											if(itemLen == 0)
											{
												Log.d(TAG,String.format("itemLen == 0"));
												//error
												break;
											}
											else if(itemLen == 1)
											{
												Log.d(TAG,String.format("itemLen == 1"));
												int offsetData = PROTOCOL_DATA_START+5+iProcess;
												DeviceInfo deviceInfo = mDeviceInfoList.get(startIndex + itemReaded);
												deviceInfo.isUsed = false;
												deviceInfo.crc8 = mBufferReceived[offsetData];
											}
											else if(itemLen > 1)
											{
												Log.d(TAG,String.format("itemLen > 1"));
												int offsetData = PROTOCOL_DATA_START+5+iProcess;
												int indexCur = startIndex + itemReaded;
												DeviceInfo deviceInfo = mDeviceInfoList.get(startIndex + itemReaded);
												int k;
												deviceInfo.isUsed = true;
												deviceInfo.addr = (mBufferReceived[offsetData+0]&0xff)|((mBufferReceived[offsetData+1]&0xff)<<8);
												deviceInfo.groupId = (mBufferReceived[offsetData+2]&0xff)|((mBufferReceived[offsetData+3]&0xff)<<8);
												for(k=0;k<16;k++)
													deviceInfo.name[k] = mBufferReceived[offsetData+4+k];
												deviceInfo.lightBrightness = mBufferReceived[offsetData+20];
												deviceInfo.lightYellow = mBufferReceived[offsetData+21];
												deviceInfo.lightR = mBufferReceived[offsetData+22];
												deviceInfo.lightG = mBufferReceived[offsetData+23];
												deviceInfo.lightB = mBufferReceived[offsetData+24];
												deviceInfo.crc8 = mBufferReceived[offsetData+25];
												for(k=0;k<4;k++)
													deviceInfo.reserved[k] = mBufferReceived[offsetData+26+k];
											}
											iProcess += itemLen + 1;
											itemReaded++;
											Log.d(TAG,String.format("iProcess=%d,ReadedLength=%d",iProcess,ReadedLength));
										}
										if(startIndex + itemReaded < DEVICE_MAX_NUM )
										{
											////not over ,read next one
											OutputStream out = socket.getOutputStream();
											startIndex += itemReaded;
											int maxReadLen = 256 - 2 - 4;
											mBufferSend[PROTOCOL_DATA_START+0] = (byte)startIndex;
											mBufferSend[PROTOCOL_DATA_START+1] = (byte)(startIndex>>8);
											mBufferSend[PROTOCOL_DATA_START+2] = (byte)maxReadLen;
											mBufferSend[PROTOCOL_DATA_START+3] = (byte)(maxReadLen>>8);
											int len = fillSendBufferCmd(mBufferSend,0x0002,4);
											
											out.write(mBufferSend,0,len);
											out.flush();
										}
										else
										{
											//read ok
										}
									
									}
									break;
								}
							}
						}
						catch(Exception e){

						}
					}

				}


				/////////////sleep////////////////////
				try {
					Thread.sleep(mDealyTime);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			
		}
		
	}
	public class DeviceInfo
	{
		boolean isUsed;
		int addr;
		int groupId;
		byte lightBrightness;
		byte lightYellow;
		byte lightR;
		byte lightG;
		byte lightB;
		byte crc8;
		byte[] name;
		byte[] reserved;
	};
	public class GroupInfo
	{
		int groupId;
		byte[] groupName;
	};
	public class ItemInfo
	{
		public static final int TYPE_LIGHT = 0;
		public static final int TYPE_GROUP = 1;
		int type;
		int id;
		String name;
	};
}
