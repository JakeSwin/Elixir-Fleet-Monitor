defmodule FleetMonitor.Runtime.Server do
  use GenServer
  require Logger

  @type t :: pid

  @command "python3 " <> Path.expand("./lib/python/fleet_monitor.py") <> " /camera/image_raw"
  @me __MODULE__

  ### Client Process

  def start_link(_) do
    GenServer.start_link(__MODULE__, nil, name: @me)
  end

  def get_image do
    GenServer.call(@me, { :get_image })
  end

  def subscribe(pid) do
    GenServer.call(@me, { :subscribe, pid })
  end

  ### Server Process
  def init(_) do
    _port = Port.open({:spawn, @command}, [:binary, :exit_status])

    { :ok, %{ latest_image: nil, exit_status: nil, image_partial: nil, pids: [] }}
  end

  def handle_info({ _port, { :data, "data: " <> image_data }}, state) do
    # Logger.info "Latest output: #{image_data}"
    # Logger.info "Image Received"
    Enum.each(state.pids, fn pid ->
      send(pid, { :new_image, state.image_partial })
    end)
    # Enum.each(state.pids, fn pid ->
    #   send(pid, { :test , Enum.random([1,2,3,4]) })
    # end)
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

  def handle_call({ :subscribe, pid }, _from, state) do
    { :reply, :ok, %{ state | pids: [ pid | state.pids ] }}
  end
end
