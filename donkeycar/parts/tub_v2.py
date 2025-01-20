import os
import csv
import numpy as np
from PIL import Image
import logging

from donkeycar.parts.datastore_v2 import Manifest, ManifestIterator


logger = logging.getLogger(__name__)


class Tub(object):
    """
    A datastore to store sensor data in a key, value format.
    It will write image, steering, and throttle data to a CSV file and store images with a specific naming convention.
    """

    def __init__(self, base_path, inputs=[], types=[], metadata=[],
                 max_catalog_len=1000, read_only=False):
        self.base_path = base_path
        self.images_base_path = os.path.join(self.base_path, Tub.images())
        self.csv_file_path = os.path.join(self.base_path, 'tub_data.csv')
        self.inputs = inputs
        self.types = types
        self.metadata = metadata
        self.manifest = Manifest(base_path, inputs=inputs, types=types,
                                 metadata=metadata, max_len=max_catalog_len,
                                 read_only=read_only)
        self.input_types = dict(zip(self.inputs, self.types))

        # Create images folder if necessary
        if not os.path.exists(self.images_base_path):
            os.makedirs(self.images_base_path, exist_ok=True)

        # Create or open CSV file and write headers only if it's a new file
        if not os.path.exists(self.csv_file_path):
            with open(self.csv_file_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['image_id', 'steering', 'throttle'])  # Add headers for the CSV file

    def write_record(self, record=None):
        """
        Handles writing steering, throttle data to CSV and saving images.
        """
        contents = dict()
        for key, value in record.items():
            if value is None:
                continue
            elif key not in self.input_types:
                continue
            else:
                input_type = self.input_types[key]
                if input_type == 'float':
                    contents[key] = float(value)
                elif input_type == 'str':
                    contents[key] = value
                elif input_type == 'int':
                    contents[key] = int(value)
                elif input_type == 'boolean':
                    contents[key] = bool(value)
                elif input_type == 'nparray':
                    contents[key] = value.tolist()
                elif input_type == 'list' or input_type == 'vector':
                    contents[key] = list(value)
                elif input_type == 'image_array':
                    # Handle image array and save the image
                    image = Image.fromarray(np.uint8(value))
                    image_id = str(self.manifest.current_index).zfill(6) + '.jpg'
                    image_path = os.path.join(self.images_base_path, image_id)
                    image.save(image_path)
                    contents[key] = image_id

                    # Append to CSV: write image_id, steering, and throttle
                    with open(self.csv_file_path, mode='a', newline='') as f:
                        writer = csv.writer(f)
                        steering = record.get('steering', 0.0)
                        throttle = record.get('throttle', 0.0)
                        writer.writerow([image_id, steering, throttle])

        # Write the record to the manifest (for internal bookkeeping)
        self.manifest.write_record(contents)

    def close(self):
        logger.info(f'Closing tub {self.base_path}')
        self.manifest.close()

    @classmethod
    def images(cls):
        return 'images'



class TubWriter(object):
    """
    A Donkey part, which can write records to the datastore.
    """
    def __init__(self, base_path, inputs=[], types=[], metadata=[],
                 max_catalog_len=1000):
        self.tub = Tub(base_path, inputs, types, metadata, max_catalog_len)

    def run(self, *args):
        assert len(self.tub.inputs) == len(args), \
            f'Expected {len(self.tub.inputs)} inputs but received {len(args)}'
        record = dict(zip(self.tub.inputs, args))
        self.tub.write_record(record)
        return self.tub.manifest.current_index

    def __iter__(self):
        return self.tub.__iter__()

    def close(self):
        self.tub.close()

    def shutdown(self):
        self.close()


class TubWiper:
    """
    Donkey part which deletes a bunch of records from the end of tub.
    This allows to remove bad data already during recording. As this gets called
    in the vehicle loop the deletion runs only once in each continuous
    activation. A new execution requires to release of the input trigger. The
    action could result in a multiple number of executions otherwise.
    """
    def __init__(self, tub, num_records=20):
        """
        :param tub: tub to operate on
        :param num_records: number or records to delete
        """
        self._tub = tub
        self._num_records = num_records
        self._active_loop = False

    def run(self, is_delete):
        """
        Method in the vehicle loop. Delete records when trigger switches from
        False to True only.
        :param is_delete: if deletion has been triggered by the caller
        """
        # only run if input is true and debounced
        if is_delete:
            if not self._active_loop:
                # action command
                self._tub.delete_last_n_records(self._num_records)
                # increase the loop counter
                self._active_loop = True
        else:
            # trigger released, reset active loop
            self._active_loop = False