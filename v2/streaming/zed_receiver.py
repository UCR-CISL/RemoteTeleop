import pyzed.sl as sl
import cv2

def main():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    
    # Critical: Tell the SDK to listen to the network stream
    input_type = sl.InputType()
    # Use "127.0.0.1" if testing on the same machine
    input_type.set_from_stream("127.0.0.1", 30000) 
    init_params.input = input_type

    print("Attempting to connect to stream...")
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Connection failed. Is the sender running?")
        return

    image = sl.Mat()
    while True:
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left image from the stream
            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
            
            cv2.imshow("ZED Stream Receiver", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()