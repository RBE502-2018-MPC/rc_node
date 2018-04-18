import csv


def load_file(file_name):
    x = []
    y = []
    with open(file_name, 'rt') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',')
        for row in spamreader:
            # print(', '.join(row))
            x.append(float(row[0]))
            y.append(float(row[1]))
    position = [x, y]
    return position
