import numpy as np
import cv2

def read_flo_file(filename):
    with open(filename, 'rb') as f:
        magic = np.fromfile(f, np.float32, count=1)
        if magic != 202021.25:
            raise Exception('Invalid .flo file format')
        width, height = np.fromfile(f, np.int32, count=2)
        data = np.fromfile(f, np.float32, count=2 * width * height)
    flow = np.resize(data, (height, width, 2))
    return flow[:, :, 0], flow[:, :, 1]

def compute_flow_norm_and_angle(flow_x, flow_y):
    flow_norm = np.sqrt(flow_x ** 2 + flow_y ** 2)
    flow_angle = np.arctan2(flow_y, flow_x) * 180. / np.pi
    flow_angle[flow_angle < 0] += 360
    return flow_norm, flow_angle

def flow_to_color(flow_norm, flow_angle, max_flow=None):
    if max_flow is not None:
        flow_norm = np.clip(flow_norm / max_flow, 0, 1)
    else:
        max_norm = np.percentile(flow_norm, 99)
        flow_norm = np.clip(flow_norm / max_norm, 0, 1)

    hsv = np.zeros((flow_norm.shape[0], flow_norm.shape[1], 3), dtype=np.uint8)
    hsv[..., 0] = flow_angle / 2  # Hue
    hsv[..., 1] = 255             # Saturation
    hsv[..., 2] = cv2.normalize(flow_norm, None, 0, 255, cv2.NORM_MINMAX)  # Value
    rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return rgb

def visualize_flow(filename, save_path):
    flow_x, flow_y = read_flo_file(filename)
    flow_norm, flow_angle = compute_flow_norm_and_angle(flow_x, flow_y)
    color_flow = flow_to_color(flow_norm, flow_angle)

    # Uncomment these lines if you are sure your environment supports GUI
    # cv2.imshow('Optical Flow', color_flow)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    print("Saving the visualized flow image...")
    cv2.imwrite(save_path, color_flow)
    print("Image saved at:", save_path)


# Example usage
visualize_flow('/home/anuj/Desktop/spynet-pytorch-main/pytorch-spynet-master/out.flo', 
               '/home/anuj/Desktop/spynet-pytorch-main/pytorch-spynet-master/visualized_flow.png')

