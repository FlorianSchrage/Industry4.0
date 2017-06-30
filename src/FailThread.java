public class FailThread extends Thread
{
	private SenderThread sender;
	private ReceiverThread receiver;
	private Robot apiRef;
	private int delay_restartCommunicationAfterFailure;
	
	public FailThread(Robot apiRef, SenderThread sender, ReceiverThread receiver, int delay_restartCommunicationAfterFailure)
	{
		this.apiRef = apiRef;
		this.sender = sender;
		this.receiver = receiver;
		this.delay_restartCommunicationAfterFailure = delay_restartCommunicationAfterFailure;
	}
	
	public void run()
	{
		apiRef.print("Stopping Communication");
		sender.terminate();
		try
		{
			sender.join();
		} catch (InterruptedException e)
		{
			e.printStackTrace();
		}
		apiRef.print("Sender terminated");
		apiRef.moveToOOOPlace();
		receiver.terminate();
		try
		{
			receiver.join();
		} catch (InterruptedException e)
		{
			e.printStackTrace();
		}
		apiRef.print("Receiver terminated");
		
		try
		{
			Thread.sleep(delay_restartCommunicationAfterFailure);
		} catch (InterruptedException e)
		{
			e.printStackTrace();
		}
		apiRef.restartCommunication();
	}
}
