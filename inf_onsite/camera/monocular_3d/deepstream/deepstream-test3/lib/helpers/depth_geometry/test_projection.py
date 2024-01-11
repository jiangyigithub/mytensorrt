import unittest
from projection import projection_init

class TestProjection(unittest.TestCase):

    def test_add_positive_numbers(self):
        camera_pose_info =[[-1.740657, 0.030855, 0.029865],[20.169886, -14.505331, 6.657059]]
        result = projection_init(camera_pose_info)
        self.assertTrue(result)


if __name__ == '__main__':
    unittest.main()