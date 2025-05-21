all: cluonDataStructures_pb2.py opendlv_message_standard_pb2.py

cluonDataStructures.proto: cluonDataStructures.odvd
	cluon-msc --proto --out=cluonDataStructures.proto cluonDataStructures.odvd

cluonDataStructures_pb2.py: cluonDataStructures.proto
	protoc --python_out=. cluonDataStructures.proto

opendlv-message-standard.proto: opendlv-message-standard-1.0.odvd
	cluon-msc --proto --out=opendlv-message-standard.proto opendlv-message-standard-1.0.odvd

opendlv_message_standard_pb2.py: opendlv-message-standard.proto
	protoc --python_out=. opendlv-message-standard.proto

clean:
	rm -f *_pb2.py *pyc *.proto 
