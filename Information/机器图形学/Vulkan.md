Vulkan 是 OpenGL 的下一代, 为了并行硬件优化的图形接口. Vulkan 特点是自由度比 OpenGL 更大, 但是开发难度也更高.

## Vulkan Concept Objects 

- `VkInstance`: Vulkan Context, 存储所有的全局状态.
- `VkPhysicalDevice`: GPU, physical details
- `VkDevice`: Logical GPU 
- `VkBuffer`: chunk of GPU visible memory 
- `VkImage`: Texture to write to and read from 
- `VkPipeline`: State of GPU needed to draw, like: shaders, rasterization options, depth
- `VkRenderPass`: all dwaring commands have to be done inside a rederpass 
- `VkFrameBuffer`: the target images for a renderpass. 
- `VkCommandBuffer`: encoded GPU commands. 
- `VkQueue`: GPUs will have a set of queues with different properties 
- `VkDescriptorSet`: holds the binding information that connects shader inputs to data such as `VkBuffer` resoruces and `VkImage` textures. 
- `VkSwapchainKHR`: holds the images for the screen. 
- `VkSemaphore`: synchornoizes GPU to GPU execution of commands. 
- `VkFence`: synchronizes GPU to GPu execution of commands. 

## Vulkan Flow

### Initialization 

1. 创建一个 `VkInstance`
2. 从 `VkInstance` 中, 请求 `VkPhysicalDevice` (实际物理 GPU) 列表 
3. 选用某个 `VkPhysicalDevice`, 用其创建一个 `VkDevice`
4. 从 `VkDevice` 中获取一些 `VkQueue` 句柄, 用于执行实际命令.
5. 初始化 `VkSwapchainKHR`, `VkCommandPool` 等对象

## Asset Initialization 

## Render Loop 

```cpp
// ask the swapchain of the index of the swapchain image we can reder onto
int image_index = request_image(mySwapChain)

VkCommandBuffer cmd = allocate_command_buffer();
vkBeginCommandBuffer(cmd, ...);

//each framebuffer refers to a image in the swapchain
vkCmdBeginRenderPass(cmd, main_render_pass, framebuffers[image_index]);

for (obj in PassObjects) {
	// bind shaders and configuration used to render the obj
	vkCmdBindPipeline(cmd, obj.pipeline);

	// bind the vertex and index buffers for rendering the object
	vkCmdBindVertexBuffers(cmd, obj.VertexBuffer, ...);
	vkCmdIndexBuffer(cmd, obj.IndexBuffer, ...);

	// bind the descriptor sets for the obj (shader input)
	vkCmdBindDescriptorSets(cmd, obj.textureDescriptorSet);
	vkCmdBindDescriptorSets(cmd, obj.parametersDescriptorSet);

	// execute drawing
	vkCmdDraw(cmd, ...);
}

// finalize the render pass and command buffer
vkCmdEndRenderPass(cmd);
vkEndCommandBuffer(cmd);

// submit the command buffer to begin execution on GPU
vkQueueSubmit(graphicsQueue, cmd, ...);

// display the image we just rendered on the screen. 
// semaphore makes sure the image isn't presented until cmd is finished executing 
vkQueuePresent(graphicsQueue, renderSemaphore);
```

## 参考

https://vkguide.dev/docs/introduction/vulkan_execution/
