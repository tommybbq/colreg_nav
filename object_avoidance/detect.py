import math
from dataclasses import dataclass

import torch

from object_avoidance.models.common import DetectMultiBackend
from object_avoidance.utils.dataloaders import LoadImages
from object_avoidance.utils.general import (
    check_img_size,
    non_max_suppression,
    Profile,
    scale_boxes,
)
import lib.config_loader as config_loader


@dataclass
class DetectedObject:
    # columns of x0, y0, x1, y1, confidence, class id
    # image coords start at (0, 0) at top left of image
    _detections: torch.Tensor
    _y_x_shape: tuple[int, int]

    # def _inverse_mercator_latitude(self, y: torch.Tensor) -> torch.Tensor:
    #     half_y = self._y_x_shape[0] / 2
    #     # scales -1 - 1, then -pi - pi
    #     scaled_y = -(y - half_y) / half_y * torch.pi
    #     latitude = 2 * torch.arctan(torch.exp(scaled_y)) - (0.5 * torch.pi)
    #     return latitude

    def relative_headings(self) -> torch.Tensor:
        """
        Headings relative to camera direction in the range 0 - 360.
        Ricoh Theta uses Mercator projection, so x-axis is preserved.
        """
        centers = (self._detections[:, 0] + self._detections[:, 2]) / 2
        headings = 180 - (centers / self._y_x_shape[1] * 360)  # 180 -> -180 center at 0
        negative_idx = headings < 0
        headings[negative_idx] = headings[negative_idx] + 360
        return headings  # 180 -> 360 center is 0

    def estimated_distance(self, km_to_shore: float) -> torch.Tensor:
        """
        Gives estimated distance in m to a boat
        """
        relative_size = (
            self._detections[:, 2] - self._detections[:, 0]
        ) / self._y_x_shape[1]
        angular_size = relative_size * torch.pi

        # made up function that puts boats at 10m close to shore and goes to 200m
        estimated_size = 200 / (1 + math.exp(3 - km_to_shore / 10))

        distance = estimated_size / (2 * torch.arctan(angular_size / 2))
        return distance


class Detect:
    def __init__(self, config: config_loader.ConfigLoader) -> None:
        self.cfg = config.data

        # load model
        self.device = torch.device("cpu")
        self.model = DetectMultiBackend(
            "object_avoidance/yolov8s.pt",
            device=self.device,
            dnn=False,
            data="data/coco128.yaml",
            fp16=False,
        )
        self.stride, self.pt = self.model.stride, self.model.pt
        self.imgsz = check_img_size((640, 640), s=self.stride)  # check image size

        # warm up model
        bs = 1
        self.model.warmup(
            imgsz=(1 if self.pt or self.model.triton else bs, 3, *self.imgsz)
        )  # warmup

    def inference(self, source: str) -> list[list[DetectedObject | None]]:
        """
        Run object detection.

        Inputs:
            source (str): path to image file or folder of files

        Returns:
            list[list[DetectedObject | None]]: list of list that either contains DetectedObjects or is empty
        """
        dataset = LoadImages(
            source, img_size=self.imgsz, stride=self.stride, auto=self.pt, vid_stride=1
        )

        dt = (
            Profile(device=self.device),
            Profile(device=self.device),
            Profile(device=self.device),
        )
        detections = []
        for _, im, im0s, _, _ in dataset:
            with dt[0]:
                im = torch.from_numpy(im).to(self.model.device)
                im = im.half() if self.model.fp16 else im.float()  # uint8 to fp16/32
                im /= 255  # 0 - 255 to 0.0 - 1.0
                if len(im.shape) == 3:
                    im = im[None]  # expand for batch dim
                if self.model.xml and im.shape[0] > 1:
                    ims = torch.chunk(im, im.shape[0], 0)

            # Inference
            with dt[1]:
                if self.model.xml and im.shape[0] > 1:
                    pred = None
                    for image in ims:
                        if pred is None:
                            pred = self.model(
                                image, augment=False, visualize=False
                            ).unsqueeze(0)
                        else:
                            pred = torch.cat(
                                (
                                    pred,
                                    self.model(
                                        image, augment=False, visualize=False
                                    ).unsqueeze(0),
                                ),
                                dim=0,
                            )
                    pred = [pred, None]
                else:
                    pred = self.model(im, augment=False, visualize=False)

            # NMS
            with dt[2]:
                pred = non_max_suppression(
                    pred,
                    self.cfg.conf_thres,
                    self.cfg.iou_thres,
                    None,
                    self.cfg.agnostic_nms,
                    max_det=self.cfg.max_det,
                )

            # Process predictions
            detected = []
            for all_det in pred:  # per image
                det = all_det[all_det[:, -1] == 8]
                im0 = im0s.copy()
                im_shape = im0.shape

                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im_shape).round()
                    detected.append(DetectedObject(det, im_shape[:2]))

            detections.append(detected)

        return detections


if __name__ == "__main__":
    cv_cfg = config_loader.ConfigLoader().load_file("config/object_avoidance.toml")
    detector = Detect(cv_cfg)
    results = detector.inference("../../../../Pictures/sea++/base_images/R0010233.JPG")
    # results = detector.inference("../../../python/yolov5/paddle_boat.jpg")
    for result in results:
        for detection in result:
            if detection:
                print(detection.relative_headings())
                print(detection.estimated_distance(0))
