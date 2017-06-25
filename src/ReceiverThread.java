import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;
import java.util.concurrent.Semaphore;

public class ReceiverThread extends Thread
{
	private volatile boolean running = true;
	private Robot apiRef;
	private Semaphore sema;
	private Socket socket;
	private BufferedReader networkIn;
	
	public ReceiverThread(Robot apiRef, Semaphore sema)
	{
		this.apiRef = apiRef;
		this.sema = sema;
	}
	
	public void terminate()
	{
		running = false;
	}
	
	public void run()
	{
		while(running)
		{
			try
			{
				sema.acquire();
			} catch (InterruptedException e)
			{
				e.printStackTrace();
			}
			
			apiRef.print("Receiver aquired");
			Socket newSocket = apiRef.getSocket();
//			Socket newSocket= dummyGetSocket();

			try
			{
				if(newSocket != socket)
				{
					apiRef.print("Receiver new Socket");
					socket = newSocket;
					networkIn = new BufferedReader(new InputStreamReader(socket.getInputStream()));
				}

				apiRef.print("Checking for Data");
				if(networkIn.ready())
				{
					String line = networkIn.readLine();
					apiRef.print("Received " + line);
					apiRef.interpretJsonString(line);
				}

			} catch(IOException e)
			{
				e.printStackTrace();
			}
			finally
			{
				sema.release();
			}
		}
	}
	
	
//	public Socket dummyGetSocket()
//	{
//		if(socket == null || socket.isClosed() || !socket.isConnected())
//		{			
//			try
//			{
//				socket = new Socket("192.168.1.10", 33333);
//				apiRef.print("Socket 33333 successful");
//			} catch (UnknownHostException e)
//			{
//				apiRef.print("E: unknown host");
//				e.printStackTrace();
//			} catch (IOException e)
//			{
//				apiRef.print(e.getMessage());
//				e.printStackTrace();
//			}
//		}
//		
//		return socket;
//	}
}
