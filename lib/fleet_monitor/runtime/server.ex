defmodule FleetMonitor.Runtime.Server do
  use GenServer
  require Logger

  @type t :: pid

  @command "python3.8 ./lib/python/fleet_monitor.py"
  @me __MODULE__

  ### Client Process

  def start_link(_) do
    GenServer.start_link(__MODULE__, nil, name: @me)
  end

  def get_image do
    GenServer.call(@me, { :get_image })
  end

  ### Server Process
  def init(_) do
    _port = Port.open({:spawn, @command}, [:binary, :exit_status])

    { :ok, %{ latest_image: nil, exit_status: nil }}
  end

  def handle_info({ _port, { :data, "data " <> image_data }}, state) do
    # Logger.info "Latest output: #{image_data}"
    { :noreply, %{ state | latest_image: image_data }}
  end

  def handle_info({ _port, { :data, data }}, state) do
    Logger.info "Info: #{data}"
    { :noreply, state }
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
