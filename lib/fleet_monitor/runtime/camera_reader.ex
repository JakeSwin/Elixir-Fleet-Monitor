defmodule FleetMonitor.Runtime.CameraReader do
  use GenServer
  require Logger
  import String, only: [to_atom: 1]

  @type t :: pid

  @python_version "python3.8"
  @python_path Path.join(:code.priv_dir(:fleet_monitor), "/python/camera_reader.py")
  @me __MODULE__

  ### Client Process

  def start_link([image_topic: topic] = args) do
    GenServer.start_link(__MODULE__, args, name: to_atom(topic))
  end

  def get_image(topic) do
    GenServer.call(to_atom(topic), { :get_image })
  end

  ### Server Process
  def init(image_topic: topic) do
    _port = Port.open(
      {:spawn, Enum.join([@python_version, @python_path, topic], " ")},
      [:binary, :exit_status]
    )

    { :ok, %{ latest_image: nil, exit_status: nil, image_partial: nil, topic: topic }}
  end

  def handle_info({ _port, { :data, "data: " <> image_data }}, state) do
    # Logger.info "Latest output: #{image_data}"
    # Logger.info "Image Received"
    if state.image_partial do
      Phoenix.PubSub.broadcast!(FleetMonitor.PubSub, state.topic, {:new_image, state.image_partial})
    end
    { :noreply, %{ state | latest_image: state.image_partial, image_partial: image_data }}
  end

  def handle_info({ _port, { :data, "info: " <> _info_data }}, state) do
    # Logger.info "Info: #{info_data}"
    { :noreply, state }
  end

  def handle_info({ _port, { :data, data }}, state) do
    # Logger.info "Info: #{data}"
    # Logger.info "Image Part Received"
    { :noreply, %{ state | image_partial: state.image_partial <> data } }
  end

  def handle_info({ _port, { :exit_status, status}}, state) do
    Logger.info "External exit: :exit_status: #{status}"

    { :noreply, %{ state | exit_status: status }}
  end

  def handle_info(_msg, state), do: { :noreply, state }

  def handle_call({ :get_image }, _from, state = %{ latest_image: data }) do
    image_base64 = case data do
      nil -> nil
      data -> data
    end
    { :reply, image_base64, state}
  end
end
