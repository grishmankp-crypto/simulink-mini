# 1. Use a lightweight Python 3.12 image
FROM python:3.12-slim

# 2. Install C++ build tools, CMake, and the JSON library
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# 3. Set the working directory for the container
WORKDIR /app

# 4. Copy your entire project into the container
COPY . .

# 5. Build the C++ engine and pybind11 binaries
RUN mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make

# 6. Move into the backend directory and install Python dependencies
WORKDIR /app/backend
RUN pip install --no-cache-dir -r requirements.txt

# 7. Expose port 8000 for the cloud provider
EXPOSE 8000

# 8. Start the FastAPI server (binding to 0.0.0.0 is required for cloud hosting)
CMD ["uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8000"]