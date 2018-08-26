from SimpleCV import (
    Camera,
    Color,
    Display,
    DrawingLayer
    )


class SolarCamera:
    def __init__(self, window_size=(640, 480), **kwargs):
        while True:  # Initialize the Camera
            try:
                self.cam = Camera()
                self.cam.getImage().flipHorizontal()
            except:
                continue
            else:
                break

        self.image = None
        self.window_size = window_size
        self.display = Display(self.window_size)
        self.__window_center = (self.window_size[0]/2, self.window_size[1]/2)
        self.__distance = None
        self.__blobs = None
        self.__segmented = None
        self.__circles = None
        self.__scope_layer = None
        self.initialize_scope_layer()

    @property
    def is_there_sun(self):
        self.__binarize_image()  # binarize image
        if self.__blobs:
            self.__circles = self.__blobs.filter([b.isCircle(0.2) for b in self.__blobs])
            if self.__circles:
                return True
        return False

    def __binarize_image(self):
        #  dilate the image
        self.__distance = self.image.colorDistance(Color.BLACK).dilate(2)
        #  segment image with colors ranging from 250 to 255
        self.__segmented = self.__distance.stretch(250, 255)
        #  identify if there's a blob
        self.__blobs = self.__segmented.findBlobs(minsize=2000)
        
    def get_image(self):
        self.image = self.cam.getImage().flipHorizontal()
       
    def show_image(self):
        self.image.addDrawingLayer(self.__scope_layer)
        self.image.show()
                        
    def mark_sun(self):
        self.image.drawCircle(
            (self.__circles[-1].x, self.__circles[-1].y),
            self.__circles[-1].radius(), 
            Color.BLUE, 3
        )
    
    @property
    def get_sun_coordinates(self):
        return (self.__circles[-1].x, self.__circles[-1].y)
                            
    def initialize_scope_layer(self):
        self.__scope_layer = DrawingLayer(self.window_size) #  same as window size
        #  (position/coordinates, diameter, Color, Thickness of the lines)
        self.__scope_layer.circle(self.__window_center, 50, Color.BLACK, width=3)
        self.__scope_layer.circle(self.__window_center, 100, Color.BLACK, width=2)
        self.__scope_layer.line((self.__window_center[0], self.__window_center[1]-50),(self.__window_center[0], 0), Color.BLACK, width=2)
        self.__scope_layer.line((self.__window_center[0], self.__window_center[1]+50), (self.__window_center[0], self.window_size[1]), Color.BLACK, width=2)
        self.__scope_layer.line((self.__window_center[0]-50, self.__window_center[1]), (0, self.__window_center[1]), Color.BLACK, width=2)
        self.__scope_layer.line((self.__window_center[0]+50, self.__window_center[1]), (self.window_size[0], self.__window_center[1]), Color.BLACK, width=2)
            
    def print_sun_coordinates(self):
        if self.__circles:
            print("x:", self.get_sun_coordinates[0], "y:", self.get_sun_coordinates[1])
    
    @property
    def get_window_center(self):
        return self.__window_center
