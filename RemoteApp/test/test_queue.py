# import queue
# import threading

# def worker(q,worker_number):
#     while True:
#         item = q.get()
#         if item is None:
#             # q.task_done()
#             break
#         # Process the item here
#         print("worker_number :",worker_number,"Processing:", item)
#         q.task_done()

# # Create a new queue
# my_queue = queue.Queue()

# # Create worker threads
# num_threads = 2
# for j in range(num_threads):
#     threading.Thread(target=worker, args=(my_queue,j)).start()

# # Add items to the queue
# for i in range(10):
#     my_queue.put(i)

# # Wait for all tasks to be completed
# my_queue.join()

# # Signal workers to exit
# for _ in range(num_threads):
#     my_queue.put(None)

# # Wait for workers to finish
# for thread in threading.enumerate():
#     if thread != threading.current_thread():
#         thread.join()

# print("All tasks completed.")




import threading
import queue

data_queue = queue.Queue()

def producer():
    data = [1, 2, 3]
    data_queue.put(data)

def consumer(consumer_id):
    data = data_queue.get()
    print(f"Consumer {consumer_id} received data:", data)

# Create threads
producer_thread = threading.Thread(target=producer)
consumer1_thread = threading.Thread(target=consumer, args=(1,))
consumer2_thread = threading.Thread(target=consumer, args=(2,))

# Start threads
producer_thread.start()
consumer1_thread.start()
consumer2_thread.start()

# Wait for threads to finish
producer_thread.join()
consumer1_thread.join()
consumer2_thread.join()

