# Gazebo Sim Architecture

Gazebo Sim is an application entry-point of Gazebo, generally encompassing the use of all other Gazebo libraries. As an executable, it runs a simulation by launching two processes: the backend server process and frontend client process.

> Gazebo Sim 是 Gazebo 的应用程序入口点，通常包含所有其他 Gazebo 库的使用。作为可执行文件，它通过启动两个进程来运行模拟：后端服务器进程和前端客户端进程。

The core functionality of Gazebo Sim is the client-server communication and the various plugins it loads to facilitate the simulation. Plugins are libraries that are loaded at runtime. They allow Gazebo Sim to utilize other Gazebo libraries. There are many plugin types in the Gazebo framework. For example, there are plugins that introduce new physics engines or rendering engines. However, for the purpose of this document, any mention of plugins is referring only to Gazebo Sim server and GUI plugins.

> Gazebo Sim 的核心功能是客户端-服务器通信及其加载的各种插件以促进模拟。插件是在运行时加载的库。它们允许 Gazebo Sim 使用其他 Gazebo 库。 Gazebo 框架中有很多插件类型。例如，有一些插件引入了新的物理引擎或渲染引擎。然而，就本文档而言，任何提及的插件仅指 Gazebo Sim 服务器和 GUI 插件。

Because they’re loaded at runtime, Gazebo does not need to be recompiled to add or remove plugins. Gazebo Sim ships with many plugins by default (Server plugins, Gazebo GUI plugins, Gazebo Sim GUI plugins, and more), all of which are optional and can be removed by the user. Users can also add more plugins and even write their own plugins that will be compiled into library files.

> 因为它们是在运行时加载的，所以 Gazebo 不需要重新编译来添加或删除插件。 Gazebo Sim 默认附带许多插件（服务器插件、Gazebo GUI 插件、Gazebo Sim GUI 插件等），所有这些插件都是可选的，用户可以删除。用户还可以添加更多插件，甚至编写自己的插件，这些插件将被编译成库文件。

Gazebo libraries are modular. Plugins let Gazebo Sim utilize other libraries. For example, Gazebo Physics and Gazebo Sim are independent of one another, so Gazebo Sim has a physics plugin that uses Gazebo Physics as a library and incorporates Gazebo specifics, allowing Physics to be a system running in the simulation loop.

> Gazebo 库是模块化的。插件让 Gazebo Sim 利用其他库。例如，GazeboPhysics 和 GazeboSim 是相互独立的，因此 GazeboSim 有一个物理插件，该插件使用 GazeboPhysics 作为库，并结合了 Gazebo 的具体细节，使 Physics 成为一个在模拟循环中运行的系统。

The Gazebo Physics library is only used in the physics plugin, not in other plugins nor in the core of Gazebo Sim. Some libraries are only used by one plugin, or one in each process (frontend and backend). However, some foundational libraries are used by all plugins, like Gazebo Common which provides common functionality like logging, manipulating strings, file system interaction, etc., to all plugins. Other such libraries include Gazebo Plugin, Gazebo Math, SDFormat, and more.

> Gazebo 物理库仅在物理插件中使用，不在其他插件或 Gazebo Sim 的核心中使用。有些库仅由一个插件使用，或者每个进程（前端和后端）中都有一个。但是，所有插件都使用一些基础库，例如 Gazebo Common，它为所有插件提供通用功能，例如日志记录、操作字符串、文件系统交互等。其他此类库包括 Gazebo Plugin、Gazebo Math、SDFormat 等。

There are certain plugins in both the frontend and backend processes that are loaded by default in every version of Gazebo. Many other plugins are optional. One common optional plugin is the sensors plugin. Both optional and default plugins can be removed or added at any time; Gazebo Sim will continue to run with limited functionality. These demos on Server Configuration and GUI Configuration showcase that functionality.

> 每个版本的 Gazebo 中都会默认加载前端和后端进程中的某些插件。许多其他插件是可选的。一种常见的可选插件是传感器插件。可选插件和默认插件都可以随时删除或添加； Gazebo Sim 将继续运行，但功能有限。这些有关服务器配置和 GUI 配置的演示展示了该功能。

The simulation process is depicted in the diagram below, and further explained in the Backend and Frontend process sections that follow.

> 模拟过程如下图所示，并在后面的后端和前端过程部分中进一步解释。

## Gazebo Sim architecture diagram

### Backend server process

后端服务器进程

Gazebo Sim is responsible for loading plugins in the backend, referred to as systems. The server runs an entity-component system architecture (see Gazebo Sim terminology). The backend will usually have multiple systems responsible for everything in the simulation – computing physics, recording logs, receiving user commands, etc.

> Gazebo Sim 负责在后端（称为系统）加载插件。服务器运行实体组件系统架构（请参阅 Gazebo Sim 术语）。后端通常有多个系统负责模拟中的一切——计算物理、记录日志、接收用户命令等。

Systems act on entities and components of those entities. An entity is anything in the simulation scene (a model, link, light, actor, etc.), and components are their characteristics (pose, name, geometry, etc.). Server plugins have access to all entities and what components they have, and make modifications to those components. For example, a user could write a system that applies force to an entity by setting a component on that entity, and the physics system would pick that up and react by applying the force to make the entity actually move. The modification of entities and components is how server plugins communicate with each other.

> 系统作用于实体和这些实体的组件。实体是模拟场景中的任何东西（模型、链接、灯光、演员等），组件是它们的特征（姿势、名称、几何形状等）。服务器插件可以访问所有实体及其拥有的组件，并对这些组件进行修改。例如，用户可以编写一个系统，通过在实体上设置组件来向实体施加力，物理系统会拾取该组件并通过施加力来做出反应，使实体实际移动。实体和组件的修改是服务器插件相互通信的方式。

The entity component system is self-contained in Gazebo Sim, and systems act on entities and their components. Since other Gazebo Libraries don't have direct knowledge/access to entities and components, other Gazebo Libraries should be used in systems so that the server plugin can share entities (and their components) with these other Libraries. For example, the physics system shares entities and components with the Gazebo Physics library in order to achieve physics effects on entities and components.

> Gazebo Sim 中的实体组件系统是独立的，系统作用于实体及其组件。由于其他 Gazebo 库没有对实体和组件的直接知识/访问权限，因此应在系统中使用其他 Gazebo 库，以便服务器插件可以与这些其他库共享实体（及其组件）。例如，物理系统与 Gazebo 物理库共享实体和组件，以实现对实体和组件的物理效果。

There is a loop running in the backend that runs the systems, where some systems are proposing changes to entities and their components, and other systems are handling and applying those changes. This is called the “simulation loop”. The Entity Component Manager (ECM) in the backend provides the functionality for the actual querying and updating of the entities and components.

> 在运行系统的后端中有一个循环运行，其中一些系统建议对实体及其组件进行更改，而其他系统则处理和应用这些更改。这称为“模拟循环”。后端的实体组件管理器（ECM）提供实体和组件的实际查询和更新功能。

Physics, User Commands, and Scene Broadcaster are all systems launched by default in the backend. As mentioned earlier, however, even default systems can be added to or removed from the simulation loop (the Server Configuration tutorial describes how to customize default systems). For any other functionality, like sensor data processing for example, an additional system would have to be loaded. For example, if you need to generate sensor data that utilizes rendering sensors, you would need to load the sensors system. The visualization of that data, however, is left up to the client side plugins.

> 物理、用户命令和场景广播器都是在后端默认启动的系统。然而，如前所述，即使是默认系统也可以添加到模拟循环中或从模拟循环中删除（服务器配置教程介绍了如何自定义默认系统）。对于任何其他功能，例如传感器数据处理，必须加载额外的系统。例如，如果您需要生成利用渲染传感器的传感器数据，则需要加载传感器系统。然而，该数据的可视化则由客户端插件决定。

### Communication process

通信过程

Any information that crosses the process boundary (whether back-to-front with scene broadcaster or front-to-back with user commands) has to go through Gazebo Transport, the communication library. Synchronization between processes is performed by scene broadcaster, a server-side Gazebo Sim plugin that uses the Gazebo Transport and Gazebo Messages libraries to send messages from the server to the client. The messages themselves are provided by Gazebo Messages, while the framework for creating the publishers and subscribers that exchange messages is from Gazebo Transport.

> 任何跨越流程边界的信息（无论是使用场景广播器从后到前还是使用用户命令从前到后）都必须经过 Gazebo Transport（通信库）。进程之间的同步由场景广播器执行，场景广播器是一个服务器端 Gazebo Sim 插件，它使用 Gazebo Transport 和 Gazebo Messages 库将消息从服 ​​ 务器发送到客户端。消息本身由 Gazebo Messages 提供，而用于创建交换消息的发布者和订阅者的框架则来自 Gazebo Transport。

The scene broadcaster system is responsible for getting the state of the world (all of the entities and their component values) from the ever-changing simulation loop running in the back end, packaging that information into a very compact message, and periodically sending it to the frontend process. Omitting the scene broadcaster, while possible, would mean not being able to visualize data on the frontend, which can be useful for saving computational power.

> 场景广播系统负责从后端运行的不断变化的模拟循环中获取世界的状态（所有实体及其组件值），将该信息打包成非常紧凑的消息，并定期将其发送到前端进程。省略场景广播器虽然可能，但意味着无法在前端可视化数据，这对于节省计算能力很有用。

In addition to state messages sent from the scene broadcaster to the client, the client can also make requests to the server using the user commands system (another backend plugin). It also utilizes the Gazebo Transport and Gazebo Messages libraries. The user commands system has services that can be requested by the frontend GUI to insert, delete, move (etc.) models, lights, and other entities, and will send responses back to the GUI acknowledging the receipt and execution of those requests.

> 除了从场景广播器发送到客户端的状态消息之外，客户端还可以使用用户命令系统（另一个后端插件）向服务器发出请求。它还利用 Gazebo Transport 和 Gazebo Messages 库。用户命令系统具有前端 GUI 可以请求插入、删除、移动（等）模型、灯光和其他实体的服务，并将响应发送回 GUI，确认这些请求的接收和执行。

Several non-default plugins are able to send or receive messages to other processes outside of the server-client process. For example, the server-side sensors plugin publishes messages outside of the server-client process, and the server-side diff-drive plugin receives info from processes other than the client. One such process these plugins may communicate with, for example, is ROS.

> 一些非默认插件能够向服务器-客户端进程之外的其他进程发送或接收消息。例如，服务器端传感器插件在服务器客户端进程之外发布消息，服务器端 diff-drive 插件从客户端以外的进程接收信息。这些插件可能与之通信的进程之一是 ROS。

### Frontend client process

前端客户端进程

The client-side process, essentially the GUI, also comprises multiple plugins, some loaded by default, others that can be added, and all optional. All of the frontend plugins use the Gazebo GUI library. The client itself also relies on Gazebo GUI. There are visualization plugins that create the windows of the GUI, add buttons and other interactive features, and a 3D scene plugin that utilizes Gazebo Rendering that creates the scene the user sees.

> 客户端进程，本质上是 GUI，还包含多个插件，一些是默认加载的，另一些是可以添加的，并且都是可选的。所有前端插件都使用 Gazebo GUI 库。客户端本身也依赖于 Gazebo GUI。有一些可视化插件可以创建 GUI 窗口、添加按钮和其他交互功能，还有一个 3D 场景插件可以利用 Gazebo 渲染来创建用户看到的场景。

Frontend plugins typically communicate among themselves using events. Events are similar to messages, but they're processed synchronously. For example, the Render event is emitted by a 3D scene from it's rendering thread right before the scene is rendered; this gives other plugins the chance to execute code right at that thread at that moment, which is valuable to edit the 3D scene. Other such events are emitted when the user right-clicks or hovers the scene for example.

> 前端插件通常使用事件在它们之间进行通信。事件与消息类似，但它们是同步处理的。例如， Render 事件是由 3D 场景在渲染场景之前从其渲染线程发出的；这使其他插件有机会立即在该线程执行代码，这对于编辑 3D 场景很有价值。例如，当用户右键单击或悬停场景时，会发出其他此类事件。

Frontend plugins all have access to the entity and component information from the compressed message provided by the scene broadcaster system from the backend. The Entity Component Manager on the client side performs the actual message unpacking and distributes the entity and component information to the relevant plugins. This is how visualizations know how to update in accordance with backend computations. On the client side, the plugins are only reacting to the entity and component information, not interacting with it.

> 前端插件都可以从后端场景广播系统提供的压缩消息中访问实体和组件信息。客户端的实体组件管理器执行实际的消息解包，并将实体和组件信息分发给相关插件。这就是可视化如何根据后端计算进行更新的方式。在客户端，插件仅对实体和组件信息做出反应，而不与其交互。

As mentioned in the previous section, the GUI can send commands back to the server using the server’s user commands system. This would be the case, for example, if a user wanted to insert a new model through the GUI interface. User commands receives requests from the GUI and processes them, and adds those results to entities and components in the simulation loop.

> 如上一节所述，GUI 可以使用服务器的用户命令系统将命令发送回服务器。例如，如果用户想要通过 GUI 界面插入新模型，就会出现这种情况。用户命令接收来自 GUI 的请求并处理它们，并将这些结果添加到模拟循环中的实体和组件。
